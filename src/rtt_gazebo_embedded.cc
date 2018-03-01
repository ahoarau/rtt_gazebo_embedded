#include <rtt_gazebo_embedded/rtt_gazebo_embedded.hh>
#include <tinyxml.h>

using namespace RTT;
using namespace RTT::os;
using namespace std;

#if GAZEBO_MAJOR_VERSION < 6
struct g_vectorStringDup
{
  char *operator()(const std::string &_s)
  {
    return strdup(_s.c_str());
  }
};

namespace gazebo{
    bool setupServer(const std::vector<std::string> &_args)
    {
      std::vector<char *> pointers(_args.size());
      std::transform(_args.begin(), _args.end(), pointers.begin(),
                     g_vectorStringDup());
      pointers.push_back(0);
      bool result = gazebo::setupServer(_args.size(), &pointers[0]);

      // Deallocate memory for the command line arguments alloocated with strdup.
      for (size_t i = 0; i < pointers.size(); ++i)
        free(pointers.at(i));

      return result;
    }
}
#endif

RTTGazeboEmbedded::RTTGazeboEmbedded(const std::string& name)
: TaskContext(name)
// , world_path_("worlds/empty.world")
// , go_sem_(0)
// , is_paused(true)
// , gravity_vector_(3,0)
{
    this->addProperty("use_rtt_sync",use_rtt_sync_).doc("At world end, Gazebo waits on rtt's updatehook to finish (setPeriod(1) will make gazebo runs at 1Hz)");
    this->addProperty("world_path",world_path_).doc("The path to the .world file.");
    this->addProperty("sim_step_dt",sim_step_dt_).doc("The amount of time in seconds simulated at each time step (usually 1ms)");
    this->addOperation("setWorldFilePath",&RTTGazeboEmbedded::setWorldFilePath,this,RTT::OwnThread).doc("Sets the file to the world file");
    this->addOperation("add_plugin",&RTTGazeboEmbedded::addPlugin,this,RTT::OwnThread).doc("DEPRECATED, use addPlugin The path to a plugin file.");
    this->addOperation("addPlugin",&RTTGazeboEmbedded::addPlugin,this,RTT::OwnThread).doc("The path to a plugin file.");
    this->addProperty("argv",argv_).doc("argv passed to the deployer's main.");
    this->addAttribute("gravity_vector",gravity_vector_);//.doc("The gravity vector from gazebo, available after configure().");
    this->addOperation("spawnModel", &RTTGazeboEmbedded::spawnModel, this,
            RTT::OwnThread).doc(
            "The instance name of the model to be spawned and then the model name.");

    this->addOperation("getGravity", &RTTGazeboEmbedded::getGravity, this,
            RTT::OwnThread).doc(
            "Get the gravity vector");

    this->addOperation("getMaxStepSize", &RTTGazeboEmbedded::getMaxStepSize, this,
            RTT::OwnThread).doc(
            "Get the step size for the simulation (1ms default)");

    this->addOperation("listModels", &RTTGazeboEmbedded::listModels, this,
            RTT::OwnThread).doc(
            "List all models in the world");

    this->addOperation("resetModelPoses", &RTTGazeboEmbedded::resetModelPoses,
            this, RTT::OwnThread).doc("Resets the model poses.");

    this->addOperation("resetWorld", &RTTGazeboEmbedded::resetWorld,
                this, RTT::ClientThread).doc("Resets the entire world and time.");

    this->addOperation("toggleDynamicsSimulation",
            &RTTGazeboEmbedded::toggleDynamicsSimulation, this, RTT::ClientThread).doc(
            "Activate or Deactivate the physics engine of Gazebo.");

    this->addOperation("insertModelFromURDF",
            &RTTGazeboEmbedded::insertModelFromURDF, this, RTT::OwnThread).doc(
            "Insert a model from URDF. You have to make sur the meshes can be loaded.\n\n export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/the/package/containing_meshes");


    this->addOperation("insertModelFromURDFString",
            &RTTGazeboEmbedded::insertModelFromURDFString, this, RTT::OwnThread).doc(
            "Insert a model from a URDF string (from /robot_description for example). You have to make sur the meshes can be loaded.\n\n export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/the/package/containing_meshes");

    gazebo::printVersion();
}

std::vector<double> RTTGazeboEmbedded::getGravity()
{
    if (!world_)
    {
        RTT::log(RTT::Error)
                << "The world pointer was not yet retrieved. This needs to be done first, in order to be able to call this operation."
                << RTT::endlog();
        return {0,0,0};
    }
#if GAZEBO_MAJOR_VERSION > 8
    auto grav = world_->Gravity();
#else
    auto grav = world_->GetPhysicsEngine()->GetGravity();
#endif
    gravity_vector_[0] = grav[0];
    gravity_vector_[1] = grav[1];
    gravity_vector_[2] = grav[2];
    return gravity_vector_;
}

double RTTGazeboEmbedded::getMaxStepSize()
{
    if (!world_)
    {
        RTT::log(RTT::Error)
                << "The world pointer was not yet retrieved. This needs to be done first, in order to be able to call this operation."
                << RTT::endlog();
        return 0;
    }
#if GAZEBO_MAJOR_VERSION > 8
    sim_step_dt_ = world_->Physics()->GetMaxStepSize();
#else
    sim_step_dt_ = world_->GetPhysicsEngine()->GetMaxStepSize();
#endif
    return sim_step_dt_;
}

void RTTGazeboEmbedded::listModels()
{
    if (!world_)
    {
        RTT::log(RTT::Error)
                << "The world pointer was not yet retrieved. This needs to be done first, in order to be able to call this operation."
                << RTT::endlog();
        return;
    }
#if GAZEBO_MAJOR_VERSION > 8
    for(auto model : world_->Models())
        std::cout << "  - " << model->GetName() << std::endl;
#else
    for(auto model : world_->GetModels())
        std::cout << "  - " << model->GetName() << std::endl;
#endif
}

bool RTTGazeboEmbedded::insertModelFromURDF(const std::string& urdf_url)
{
    log(RTT::Info) << "Inserting model file " << urdf_url << endlog();

    TiXmlDocument doc(urdf_url);
    doc.LoadFile();
    return insertModelFromTinyXML(static_cast<void *>(&doc));
}

bool RTTGazeboEmbedded::insertModelFromURDFString(const std::string& urdf_str)
{
    TiXmlDocument doc;
    doc.Parse(urdf_str.c_str());
    return insertModelFromTinyXML(static_cast<void *>(&doc));
}

bool RTTGazeboEmbedded::insertModelFromTinyXML(void * tiny_xml_doc)
{
    if (!world_)
    {
        log(RTT::Error)
            << "The world pointer was not yet retrieved. This needs to be done first, in order to be able to call this operation."
            << endlog();
        return false;
    }
    
    if(!tiny_xml_doc)
    {
        log(RTT::Error) << "Document provided is null" << endlog();
        return false;
    }

    TiXmlDocument * doc = static_cast<TiXmlDocument*>(tiny_xml_doc);
    std::string robot_name;

    TiXmlElement* robotElement = doc->FirstChildElement("robot");

    if(robotElement)
    {
        if (robotElement->Attribute("name"))
        {
            robot_name = robotElement->Attribute("name");
        }
        else
        {
            RTT::log(RTT::Warning)
                << "Could not get the robot name in the URDF "
                << RTT::endlog();
        return false;
        }
    }
    else
    {
        RTT::log(RTT::Warning)
                << "Could not get the robot tag in the URDF "
                << RTT::endlog();
        return false;
    }

    if (robot_name.empty())
    {
        RTT::log(RTT::Warning)
                << "Could not read robot name"
                << RTT::endlog();
        return false;
    }

    TiXmlPrinter printer;
    printer.SetIndent( "    " );
    doc->Accept( &printer );
    std::string xmltext = printer.CStr();

    log(RTT::Info) << "Inserting model " << robot_name << " in the current world" << endlog();
    log(RTT::Debug) << "Inserting URDF String " << xmltext << endlog();
    world_->InsertModelString(xmltext);

    std::atomic<bool> do_exit(false);

    auto th = std::thread([&]()
    {
        int i=40;
        while(--i)
        {
            if(do_exit)
                return;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "\x1B[33mTo make it work you need first to set the path to the repo containing the meshes : (ex path ABOVE my_robot_description)\e[0m" << std::endl;
        std::cout << "\x1B[33mexport GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/the/package_above\e[0m" << std::endl;
        std::cout << "\x1B[33m----> otherwise Gazebo won't be able to read the URDF and will get stuck at this message !\e[0m" << std::endl;
    });
    // make the service call to pause gazebo
    // bool is_paused = world->IsPaused();
    // if (!is_paused) world->SetPaused(true);

    for (size_t i = 0; i < 10; i++)
    {
        log(RTT::Debug) << "Runing the world once... " << endlog();
        gazebo::runWorld(world_,1);
        log(RTT::Debug) << "Done runing the world once, verifying if the model is correctly loaded..." << endlog();
#if GAZEBO_MAJOR_VERSION > 8
        auto model = world_->ModelByName(robot_name);
#else
        auto model = world_->GetModel(robot_name);
#endif
        if(model)
        {
            do_exit = true;
            if(th.joinable())
                th.join();
            log(RTT::Info) << "Model " << robot_name << " successfully loaded" << endlog();
            // resume paused state before this call
            // world->SetPaused(is_paused);
            return true;
        }
        log(RTT::Debug) << "Model not yet loaded, trial " << i <<"/10" << endlog();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    // resume paused state before this call
    // world->SetPaused(is_paused);
    return false;

}

void RTTGazeboEmbedded::addPlugin(const std::string& filename)
{
    gazebo::addPlugin(filename);
}

void RTTGazeboEmbedded::setWorldFilePath(const std::string& file_path)
{
    if(std::ifstream(file_path))
        world_path_ = file_path;
    else
        log(RTT::Error) << "File "<<file_path<<"does not exists."<< endlog();
}

bool RTTGazeboEmbedded::resetModelPoses()
{
    if (!world_)
    {
        log(RTT::Error)
            << "The world pointer was not yet retrieved. This needs to be done first, in order to be able to call this operation."
            << endlog();
        return false;
    }

    this->world_->ResetEntities(gazebo::physics::Base::MODEL);
    return true;
}

bool RTTGazeboEmbedded::resetWorld()
{
    if (!world_)
    {
        log(RTT::Error)
            << "The world pointer was not yet retrieved. This needs to be done first, in order to be able to call this operation."
            << endlog();
        return false;
    }

    this->world_->Reset();
    return true;
}

void RTTGazeboEmbedded::OnPause(const bool _pause) {
    if (_pause) {
        if (this->isRunning()) {
            if (!is_paused_)
                this->stop();
        }
    } else {
        if (!this->isRunning()) {
            if (is_paused_)
                this->start();
        }
    }
}

bool RTTGazeboEmbedded::spawnModel(const std::string& instanceName,
        const std::string& modelName, const int timeoutSec)
{
    if (!world_)
    {
        RTT::log(RTT::Error)
                << "The world pointer was not yet retrieved. This needs to be done first, in order to be able to call this operation."
                << RTT::endlog();
        return false;
    }

    //check if file exists
    const string path = gazebo::common::SystemPaths::Instance()->FindFileURI(
            modelName);
    if (path.empty()) {
        std::cout << "\x1B[32m[[--- Model " << modelName
                << " couldn't be found ---]]\033[0m" << std::endl;
    }

    std::string model_xml;
    std::ifstream ifsURDF((path + "/model.urdf").c_str());
    if (!ifsURDF) {
        std::ifstream ifsSDF((path + "/model.sdf").c_str());
        if (!ifsSDF) {
            std::cout
                    << "\x1B[31m[[--- Can't be parsed: No model.urdf or model.sdf found! ---]]\033[0m"
                    << std::endl;
            return false;
        } else {
            model_xml.assign((std::istreambuf_iterator<char>(ifsSDF)),
                    (std::istreambuf_iterator<char>()));
        }
    } else {
        model_xml.assign((std::istreambuf_iterator<char>(ifsURDF)),
                (std::istreambuf_iterator<char>()));
    }

    TiXmlDocument gazebo_model_xml;
    gazebo_model_xml.Parse(model_xml.c_str());

    TiXmlElement* nameElement = gazebo_model_xml.FirstChildElement("robot");
    if (!nameElement) {
        cout << "it's not an urdf check for sdf" << endl;
        nameElement = gazebo_model_xml.FirstChildElement("model");
        if (!nameElement) {
            std::cout
                    << "\x1B[31m[[--- Can't be parsed: No <model> or <robot> tag found! ---]]\033[0m"
                    << std::endl;
            return false;
        } else {
            // handle sdf
            sdf::SDF root;
            root.SetFromString(model_xml);
#if GAZEBO_MAJOR_VERSION >= 6
              sdf::ElementPtr nameElementSDF = root.Root()->GetElement("model");
#else
              sdf::ElementPtr nameElementSDF = root.root->GetElement("model");
#endif
            nameElementSDF->GetAttribute("name")->SetFromString(instanceName);
        }
    } else {
        // handle urdf

        if (nameElement->Attribute("name") != NULL) {
            // removing old model name
            nameElement->RemoveAttribute("name");
        }
        // replace with user specified name
        nameElement->SetAttribute("name", instanceName);
    }

//	world->InsertModelFile(modelName);
    TiXmlPrinter printer;
    printer.SetIndent("    ");
    gazebo_model_xml.Accept(&printer);

    world_->InsertModelString(printer.CStr());

    gazebo::common::Time timeout((double) timeoutSec);

    auto modelDeployTimer(new gazebo::common::Timer());

    modelDeployTimer->Start();
    while (modelDeployTimer->GetRunning()) {
        if (modelDeployTimer->GetElapsed() > timeout) {
            gzerr
                    << "SpawnModel: Model pushed to spawn queue, but spawn service timed out waiting for model to appear in simulation under the name "
                    << instanceName << endl;
            modelDeployTimer->Stop();
            return false;
        }

        {
#if GAZEBO_MAJOR_VERSION > 8
            auto model = world_->ModelByName(instanceName);
#else
            auto model = world_->GetModel(instanceName);
#endif
            if (model){
                modelDeployTimer->Stop();
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return true;

}

bool RTTGazeboEmbedded::toggleDynamicsSimulation(const bool activate)
{
    if (!world_)
    {
        RTT::log(RTT::Error)
                << "The world pointer was not yet retrieved. This needs to be done first, in order to be able to call this operation."
                << RTT::endlog();
        return false;
    }
#if GAZEBO_MAJOR_VERSION > 8
    world_->SetPhysicsEnabled(activate);
#else
    world_->EnablePhysicsEngine(activate);
#endif
    return true;
}

bool RTTGazeboEmbedded::configureHook()
{
    log(RTT::Info) << "Creating world at "<< world_path_ <<  endlog();

    try{
        if(! gazebo::setupServer(argv_))
        {
            log(RTT::Error) << "Could not setupServer " <<  endlog();
            return false;
        }
    }catch(...){
        log(RTT::Error) << "Exception while setupping the world "  <<  endlog();
        return false;
    }

    world_ = gazebo::loadWorld(world_path_);

    if (!world_)
    {
        RTT::log(RTT::Error)
                << "Could not load the world."
                << RTT::endlog();
        return false;
    }
#if GAZEBO_MAJOR_VERSION > 8
    sim_step_dt_ = world_->Physics()->GetMaxStepSize();

    gravity_vector_[0] = world_->Gravity()[0];
    gravity_vector_[1] = world_->Gravity()[1];
    gravity_vector_[2] = world_->Gravity()[2];
#else
    sim_step_dt_ = world_->GetPhysicsEngine()->GetMaxStepSize();

    gravity_vector_[0] = world_->GetPhysicsEngine()->GetGravity()[0];
    gravity_vector_[1] = world_->GetPhysicsEngine()->GetGravity()[1];
    gravity_vector_[2] = world_->GetPhysicsEngine()->GetGravity()[2];
#endif
    n_sensors_ = 0;
#if GAZEBO_MAJOR_VERSION > 8
    for(auto model : world_->Models())
        n_sensors_ += model->GetSensorCount();
#else
    for(auto model : world_->GetModels())
        n_sensors_ += model->GetSensorCount();
#endif
    //log(RTT::Info) << "Binding world events" <<  endlog();
    world_begin_ =  gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RTTGazeboEmbedded::WorldUpdateBegin,this));
    world_end_ = gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&RTTGazeboEmbedded::WorldUpdateEnd,this));

    pause_ = gazebo::event::Events::ConnectPause(
        boost::bind(&RTTGazeboEmbedded::OnPause, this, _1));

    return true;
}


bool RTTGazeboEmbedded::startHook()
{
    if(!run_th_.joinable())
        run_th_ = std::thread(
            std::bind(&RTTGazeboEmbedded::runWorldForever,this));
    else{
        is_paused_ = false;
        unPauseSimulation();
    }
    return true;
}

void RTTGazeboEmbedded::runWorldForever()
{
    cout <<"\x1B[32m[[--- Gazebo running ---]]\033[0m"<< endl;
    gazebo::runWorld(world_, 0); // runs forever
    cout <<"\x1B[32m[[--- Gazebo exiting runWorld() ---]]\033[0m"<< endl;
}

void RTTGazeboEmbedded::updateHook()
{
    if(use_rtt_sync_)
        go_sem_.signal();
    return;
}

void RTTGazeboEmbedded::pauseSimulation()
{
    cout <<"\x1B[32m[[--- Pausing Simulation ---]]\033[0m"<< endl;
    gazebo::event::Events::pause.Signal(true);
}

void RTTGazeboEmbedded::unPauseSimulation()
{
    cout <<"\x1B[32m[[--- Unpausing Simulation ---]]\033[0m"<< endl;
    gazebo::event::Events::pause.Signal(false);
}

void RTTGazeboEmbedded::stopHook()
{
    if(!use_rtt_sync_){
        is_paused_ = true;
        pauseSimulation();
    }
}

void RTTGazeboEmbedded::WorldUpdateBegin()
{
    int tmp_sensor_count = 0;
#if GAZEBO_MAJOR_VERSION > 8    
    for(auto model : world_->Models())
        tmp_sensor_count += model->GetSensorCount();
#else
    for(auto model : world_->GetModels())
        tmp_sensor_count += model->GetSensorCount();
#endif
    do{
        if(tmp_sensor_count > n_sensors_)
        {
            if (!gazebo::sensors::load())
            {
                gzerr << "Unable to load sensors\n";
                break;
            }
            if (!gazebo::sensors::init())
            {
                gzerr << "Unable to initialize sensors\n";
                break;
            }
            gazebo::sensors::run_once(true);
            gazebo::sensors::run_threads();
            n_sensors_ = tmp_sensor_count;
        }else{
            // NOTE: same number, we do nothing, less it means we removed a model
            n_sensors_ = tmp_sensor_count;
        }
    }while(false);

    if(n_sensors_ > 0)
    {
        gazebo::sensors::run_once();
    }
}

void RTTGazeboEmbedded::WorldUpdateEnd()
{
    if(use_rtt_sync_)
        go_sem_.wait();
}

void RTTGazeboEmbedded::cleanupHook()
{
    gazebo::event::Events::sigInt.Signal();
    cout <<"\x1B[32m[[--- Stoping Simulation ---]]\033[0m"<< endl;
    if(world_)
      world_->Fini();
    cout <<"\x1B[32m[[--- Gazebo Shutdown... ---]]\033[0m"<< endl;
    //NOTE: This crashes as gazebo is running is a thread
    gazebo::shutdown();
    if(run_th_.joinable())
        run_th_.join();

    cout <<"\x1B[32m[[--- Exiting Gazebo ---]]\033[0m"<< endl;
}

ORO_CREATE_COMPONENT(RTTGazeboEmbedded)
