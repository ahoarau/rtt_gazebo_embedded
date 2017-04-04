#include <rtt_gazebo_embedded/rtt_gazebo_embedded.hh>
#include <tinyxml.h>

using namespace RTT;
using namespace RTT::os;
using namespace std;

#ifndef GAZEBO_GREATER_6
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

RTTGazeboEmbedded::RTTGazeboEmbedded(const std::string& name):
TaskContext(name),
world_path("worlds/empty.world"),
use_rtt_sync(false),
go_sem(0),
is_world_configured(false),
is_paused(true),
gravity_vector(3)
{
    log(Info) << "Creating " << name <<" with gazebo embedded !" << endlog();
    this->addProperty("use_rtt_sync",use_rtt_sync).doc("At world end, Gazebo waits on rtt's updatehook to finish (setPeriod(1) will make gazebo runs at 1Hz)");
    this->addProperty("world_path",world_path).doc("The path to the .world file.");
    this->addOperation("setWorldFilePath",&RTTGazeboEmbedded::setWorldFilePath,this,RTT::OwnThread).doc("Sets the file to the world file");
    this->addOperation("add_plugin",&RTTGazeboEmbedded::addPlugin,this,RTT::OwnThread).doc("The path to a plugin file.");
    this->addProperty("argv",argv).doc("argv passed to the deployer's main.");
    this->addConstant("gravity_vector",gravity_vector);//.doc("The gravity vector from gazebo, available after configure().");
    this->addOperation("spawn_model", &RTTGazeboEmbedded::spawnModel, this,
            RTT::OwnThread).doc(
            "The instance name of the model to be spawned and then the model name.");

    this->addOperation("resetModelPoses", &RTTGazeboEmbedded::resetModelPoses,
            this, RTT::OwnThread).doc("Resets the model poses.");

    this->addOperation("resetWorld", &RTTGazeboEmbedded::resetWorld,
                this, RTT::ClientThread).doc("Resets the entire world and time.");

    this->addOperation("toggleDynamicsSimulation",
            &RTTGazeboEmbedded::toggleDynamicsSimulation, this, RTT::ClientThread).doc(
            "Activate or Deactivate the physics engine of Gazebo.");
    gazebo::printVersion();
}

void RTTGazeboEmbedded::addPlugin(const std::string& filename)
{
    gazebo::addPlugin(filename);
}
void RTTGazeboEmbedded::setWorldFilePath(const std::string& file_path)
{
    if(std::ifstream(file_path))
        world_path = file_path;
    else
        log(RTT::Error) << "File "<<file_path<<"does not exists."<< endlog();
}

bool RTTGazeboEmbedded::resetModelPoses() {
    if (world) {
        this->world->ResetEntities(gazebo::physics::Base::MODEL);
        return true;
    } else {
        RTT::log(RTT::Warning)
                << "The world pointer was not yet retrieved. This needs to be done first, in order to be able to call this operation."
                << RTT::endlog();
        return false;
    }
}

bool RTTGazeboEmbedded::resetWorld() {
    if (world) {
        this->world->Reset();
        return true;
    } else {
        RTT::log(RTT::Warning)
                << "The world pointer was not yet retrieved. This needs to be done first, in order to be able to call this operation."
                << RTT::endlog();
        return false;
    }
}

void RTTGazeboEmbedded::OnPause(const bool _pause) {
    if (_pause) {
        if (this->isRunning()) {
            if (!is_paused)
                this->stop();
        }
    } else {
        if (!this->isRunning()) {
            if (is_paused)
                this->start();
        }
    }
}

bool RTTGazeboEmbedded::spawnModel(const std::string& instanceName,
        const std::string& modelName, const int timeoutSec) {
    if (!is_world_configured) {
        std::cout
                << "\x1B[33m[[--- You have to configure this component first! ---]]\033[0m"
                << std::endl;
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
            #ifdef GAZEBO_GREATER_6
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

    world->InsertModelString(printer.CStr());

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
            //boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());
            if (world->GetModel(instanceName)) {
                modelDeployTimer->Stop();
                break;
            }
        }
        usleep(2000);
    }

    return true;

}

bool RTTGazeboEmbedded::toggleDynamicsSimulation(const bool activate) {
    if (!is_world_configured) {
        std::cout
                << "\x1B[33m[[--- You have to configure this component first! ---]]\033[0m"
                << std::endl;
        return false;
    }
    world->EnablePhysicsEngine(activate);
    return true;
}

bool RTTGazeboEmbedded::configureHook()
{
    log(RTT::Info) << "Creating world at "<< world_path <<  endlog();

    try{
        if(! gazebo::setupServer(argv))
        {
            log(RTT::Error) << "Could not setupServer " <<  endlog();
            return false;
        }
    }catch(...){}

    world = gazebo::loadWorld(world_path);

    gravity_vector[0] = world->GetPhysicsEngine()->GetGravity()[0];
    gravity_vector[1] = world->GetPhysicsEngine()->GetGravity()[1];
    gravity_vector[2] = world->GetPhysicsEngine()->GetGravity()[2];

    is_world_configured = true;

    if(!world) return false;

    n_sensors = 0;
    for(auto model : world->GetModels())
        n_sensors += model->GetSensorCount();
    //log(RTT::Info) << "Binding world events" <<  endlog();
    world_begin =  gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RTTGazeboEmbedded::WorldUpdateBegin,this));
    world_end = gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&RTTGazeboEmbedded::WorldUpdateEnd,this));

    pause = gazebo::event::Events::ConnectPause(
        boost::bind(&RTTGazeboEmbedded::OnPause, this, _1));

    return true;
}


bool RTTGazeboEmbedded::startHook()
{
    if(!run_th.joinable())
        run_th = std::thread(
            std::bind(&RTTGazeboEmbedded::runWorldForever,this));
    else{
        is_paused = false;
        unPauseSimulation();
    }
    return true;
}

void RTTGazeboEmbedded::runWorldForever()
{
    cout <<"\x1B[32m[[--- Gazebo running ---]]\033[0m"<< endl;
    gazebo::runWorld(world, 0); // runs forever
    cout <<"\x1B[32m[[--- Gazebo exiting runWorld() ---]]\033[0m"<< endl;
}

void RTTGazeboEmbedded::updateHook()
{
    if(use_rtt_sync)
        go_sem.signal();
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
    if(!use_rtt_sync){
        is_paused = true;
        pauseSimulation();
    }
}

void RTTGazeboEmbedded::WorldUpdateBegin()
{
    int tmp_sensor_count = 0;
    for(auto model : world->GetModels())
        tmp_sensor_count += model->GetSensorCount();

    do{
        if(tmp_sensor_count > n_sensors)
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
            n_sensors = tmp_sensor_count;
        }else{
            // NOTE: same number, we do nothing, less it means we removed a model
            n_sensors = tmp_sensor_count;
        }
    }while(false);

    if(n_sensors > 0)
    {
        gazebo::sensors::run_once();
    }
}

void RTTGazeboEmbedded::WorldUpdateEnd()
{
    if(use_rtt_sync)
        go_sem.wait();
}

void RTTGazeboEmbedded::cleanupHook()
{
    gazebo::event::Events::sigInt.Signal();
    cout <<"\x1B[32m[[--- Stoping Simulation ---]]\033[0m"<< endl;
    if(world)
      world->Fini();
    cout <<"\x1B[32m[[--- Gazebo Shutdown... ---]]\033[0m"<< endl;
    //NOTE: This crashes as gazebo is running is a thread
    gazebo::shutdown();
    if(run_th.joinable())
        run_th.join();

    cout <<"\x1B[32m[[--- Exiting Gazebo ---]]\033[0m"<< endl;
}

RTTGazeboEmbedded::~RTTGazeboEmbedded()
{

}

ORO_CREATE_COMPONENT(RTTGazeboEmbedded)
