#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

class ModelPluginExample : public RTT::TaskContext
{
public:
    ModelPluginExample(std::string const& name):
    RTT::TaskContext(name)
    {
        gazebo::printVersion();
        // Connecting Events
        world_begin =  gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelPluginExample::WorldUpdateBegin,this));
        world_end   = gazebo::event::Events::ConnectWorldUpdateEnd(boost::bind(&ModelPluginExample::WorldUpdateEnd,this));
    }

    bool configureHook()
    {
#if GAZEBO_MAJOR_VERSION > 8
        model = gazebo::physics::WorldPtr()->ModelByName(getName());
#else
        model = gazebo::physics::get_world()->GetModel(getName());
#endif
        return bool(model);
    }

    void updateHook()
    {
        RTT::log(RTT::Debug)  << "UpdateHook() should do nothing ! " << RTT::endlog();
    }
    
    void WorldUpdateBegin()
    {
       RTT::log(RTT::Debug)  << getName() 
       <<" Reading cmd from RTT ports => Writing to sim model" 
       << RTT::endlog();
    }
    
    void WorldUpdateEnd()
    {
        RTT::log(RTT::Debug) << getName() 
        <<"Reading sim model => Writing to RTT ports" 
        << RTT::endlog();
    }
protected:
    gazebo::event::ConnectionPtr world_begin;
    gazebo::event::ConnectionPtr world_end;
    
    gazebo::physics::ModelPtr model;
};

ORO_CREATE_COMPONENT(ModelPluginExample)