#include "WBInterface.h"

#include "Error.h"
#include <yarp/os/ResourceFinder.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

namespace wbt {

    unsigned WBInterface::s_referenceCount = 0;

    WBInterface::WBInterface()
    : m_interface(0)
    , m_robotList(0)
    , m_configuration(0)
    , m_initialized(false)
    , m_configured(false)
    , m_dofs(-1) {}

    WBInterface::WBInterface(const WBInterface&) {}
    WBInterface::~WBInterface() {}
    WBInterface& WBInterface::operator=(const wbt::WBInterface &) { return *this; }

    WBInterface& WBInterface::sharedInstance()
    {
        static WBInterface instance;
        return instance;
        //TODO: check if the destructor get called at simulink exit
    }

    wbi::wholeBodyInterface * const WBInterface::interface() { return m_interface; }

    std::weak_ptr<wbi::iWholeBodyModel> WBInterface::model()
    {
        if (!m_model)
            return ((yarpWbi::yarpWholeBodyInterface*)m_interface)->wholeBodyModel();
        return m_model;
    }


    int WBInterface::numberOfDoFs() const { return m_dofs; }


    int WBInterface::dofsForConfigurationFileAndList(const std::string & wbiConfigFile, const std::string & list)
    {
        using namespace yarp::os;
        ResourceFinder &resourceFinder = ResourceFinder::getResourceFinderSingleton();
        resourceFinder.configure(0, 0);

        Property configurations;
        //loading defaults
        if (!configurations.fromConfigFile(resourceFinder.findFile(wbiConfigFile))) {
            return -1;
        }

        wbi::IDList jointList;

        if (!yarpWbi::loadIdListFromConfig(list, configurations, jointList)) {
            return -1;
        }
        m_dofs = jointList.size();
        return m_dofs;
    }

    bool WBInterface::configure(const std::string &robotName,
                   const std::string & localName,
                   const std::string & wbiConfigFile,
                   const std::string & list,
                   wbt::Error *error)
    {
        if (m_configured) return true;

        using namespace yarp::os;
        ResourceFinder &resourceFinder = ResourceFinder::getResourceFinderSingleton();
        resourceFinder.configure(0, 0);

        //parameters needed by this block:
        // - YARP_ROBOT_NAME: needed by resource finder for resource lookup (for now it is taken by the environment)
        // - robot: robot port. If defined overrides the one specified by wbi file
        // - moduleName: local (opened) ports.
        // - wbi config file name (default: yarpWholeBodyInterface.ini): specifies the wbi config file
        // - wbi list (default ROBOT_TORQUE_CONTROL_JOINTS): specifies the WBI list

        if (m_configuration) {
            delete m_configuration;
            m_configuration = 0;
        }
        m_configuration = new Property();
        //loading defaults
        if (!m_configuration->fromConfigFile(resourceFinder.findFile(wbiConfigFile))) {
            if (error) error->message = "Could not load the WBI configuration file";
            return false;
        }

        //overwriting values
        if (!robotName.empty())
            m_configuration->put("robot", robotName);

        m_configuration->put("localName", localName);

        //loading joint list
        if (m_robotList) {
            delete m_robotList;
            m_robotList = 0;
        }
        m_robotList = new wbi::IDList();

        if (!yarpWbi::loadIdListFromConfig(list, *m_configuration, *m_robotList)) {
            if (error) error->message = "Could not load the specified WBI list";
            return false;
        }
        m_dofs = m_robotList->size();

        m_configured = true;
        return true;
    }

    void WBInterface::clearConfiguration()
    {
        m_configured = false;
        if (m_robotList) {
            delete m_robotList;
            m_robotList = 0;
        }
        if (m_configuration) {
            delete m_configuration;
            m_configuration = 0;
        }
    }

    bool WBInterface::initialize(bool onlyModel)
    {
        //cases: 4 different possibilities
        // - yarpwbi already allocated / not allocated



        if (m_initialized) {
            s_referenceCount++;
            return true;
        }

        if (!m_configuration || !m_robotList) return false;

        if (m_interface) {
            m_interface->close();
            delete m_interface;
            m_interface = 0;
        }

        m_interface = new yarpWbi::yarpWholeBodyInterface(m_configuration->find("localName").toString().c_str(), *m_configuration);
        if (!m_interface) return false;

        s_referenceCount = 1;

        if (!m_interface->addJoints(*m_robotList)) {
            terminate();
            return false;
        }

        if (!m_interface->init()) {
            terminate();
            return false;
        }

        m_initialized = true;
        return true;
    }

    bool WBInterface::terminate()
    {
        if (s_referenceCount > 1) {
            s_referenceCount--;
            return true;
        }

        bool result = true;
        if (m_interface) {
            result = m_interface->close();
            if (result) {
                delete m_interface;
                m_interface = 0;
            }
        }

        //clean also configuration
        clearConfiguration();
        m_initialized = false;
        s_referenceCount = 0;
        return result;
    }

    bool WBInterface::isInterfaceInitialized() const { return m_initialized; }
}
