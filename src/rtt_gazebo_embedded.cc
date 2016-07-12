#include <rtt_gazebo_embedded/rtt_gazebo_embedded.hh>

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
gravity_vector(3)
{
    log(Info) << "Creating " << name <<" with gazebo embedded !" << endlog();
    this->addProperty("use_rtt_sync",use_rtt_sync).doc("At world end, Gazebo waits on rtt's updatehook to finish (setPeriod(1) will make gazebo runs at 1Hz)");
    this->addProperty("world_path",world_path).doc("The path to the .world file.");
    this->addOperation("add_plugin",&RTTGazeboEmbedded::addPlugin,this,RTT::OwnThread).doc("The path to a plugin file.");
    this->addProperty("argv",argv).doc("argv passed to the deployer's main.");
    this->addConstant("gravity_vector",gravity_vector);//.doc("The gravity vector from gazebo, available after configure().");

    gazebo::printVersion();
// #ifdef GAZEBO_GREATER_6
//     gazebo::common::Console::SetQuiet(false);
// #endif
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

    if(!world) return false;

    n_sensors = 0;
    for(auto model : world->GetModels())
        n_sensors += model->GetSensorCount();
    //log(RTT::Info) << "Binding world events" <<  endlog();
    world_begin =  gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RTTGazeboEmbedded::WorldUpdateBegin,this));
    world_end = gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&RTTGazeboEmbedded::WorldUpdateEnd,this));

    return true;
}


bool RTTGazeboEmbedded::startHook()
{
    if(!run_th.joinable())
        run_th = std::thread(std::bind(&RTTGazeboEmbedded::runWorldForever,this));
    else{
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
    if(!use_rtt_sync)
        pauseSimulation();
}

void RTTGazeboEmbedded::checkClientConnections()
{
    if(getPeerList().size() &&
        getPeerList().size() != client_map.size())
    {
        for(auto p : getPeerList())
        {
            if(client_map.find(p) == client_map.end())
            {
                if(getPeer(p)->provides("gazebo") &&
                    getPeer(p)->provides("gazebo")->hasOperation("WorldUpdateBegin") &&
                    getPeer(p)->provides("gazebo")->hasOperation("WorldUpdateEnd")
                )
                {
                    log(Info) << "Adding new client "<<p<<endlog();
                    client_map[p] = ClientConnection(getPeer(p)->provides("gazebo")->getOperation("WorldUpdateBegin"),
                                                    getPeer(p)->provides("gazebo")->getOperation("WorldUpdateEnd"));
                }
            }
        }
    }

    auto it = std::begin(client_map);
    while(it != std::end(client_map))
    {
        if(!it->second.world_end.ready() ||
            !it->second.world_begin.ready())
        {
            log(Warning) << "Removing broken connection with client "<<it->first<<endlog();
            it = client_map.erase(it);
        }else
            ++it;
    }
    return;
}

void RTTGazeboEmbedded::WorldUpdateBegin()
{
    // checkClientConnections();
    //
    // for(auto c : client_map)
    //     if(getPeer(c.first)->isConfigured()
    //         && getPeer(c.first)->isRunning())
    //         c.second.world_end_handle = c.second.world_end.send();
    //
    // for(auto c : client_map)
    //     if(getPeer(c.first)->isConfigured()
    //         && getPeer(c.first)->isRunning())
    //         c.second.world_end_handle.collect();
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
    unPauseSimulation();
    gazebo::runWorld(world, 1);
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
