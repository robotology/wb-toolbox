#include "WBInterface.h"

#include "Error.h"
#include <yarp/os/ResourceFinder.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

namespace wbt {

    unsigned WBInterface::s_referenceCount = 0;
    unsigned WBInterface::s_modelReferenceCount = 0;

    WBInterface::WBInterface()
    : m_interface(0)
    , m_model(0)
    , m_robotList(0)
    , m_configuration(0)
    , m_initialized(false)
    , m_modelInitialized(false)
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

    wbi::iWholeBodyModel * const WBInterface::model()
    {
        if (!m_model)
            return ((yarpWbi::yarpWholeBodyInterface*)m_interface)->wholeBodyModel();
        return m_model;
    }

    const yarp::os::Property * const WBInterface::currentConfiguration() const
    {
        return m_configuration;
    }


    int WBInterface::numberOfDoFs() const { return m_dofs; }

    bool WBInterface::wbdIDListFromConfigPropAndList(const yarp::os::Property& wbiConfigProp,
                                                     const std::string& list, wbi::IDList& idList) {
        // There are two ways of specifying the list throuth the mask parameter:
        // either the specified parameter is a list (in the sense of YARP configuration file list,
        // so something like (joint1,joint2,joint3) ) and it that case it is
        // considered to by directly the list of joints to load, or otherwise
        // the wbi list is just a string, and it is considered the name of the list
        // in the yarpWholeBodyInterface.ini file

        // Reset the idList
        idList.removeAllIDs();

        // Check if the list string is actually a list
        yarp::os::Value listAsValue;
        listAsValue.fromString(list.c_str());

        if (listAsValue.isList()) {
            // If the list param is a (YARP) list, load the IDList from it
            for (int jnt = 0; jnt < listAsValue.asList()->size(); jnt++)
            {
                yarp::os::ConstString jntName = listAsValue.asList()->get(jnt).asString();
                idList.addID(wbi::ID(jntName.c_str()));
            }
        } else {
            // Otherwise consider the list to be a
            if (!yarpWbi::loadIdListFromConfig(list, wbiConfigProp, idList)) {
                return false;
            }
        }
        return true;
    }

    int WBInterface::dofsForConfigurationFileAndList(const std::string & wbiConfigFile, const std::string & list)
    {
        using namespace yarp::os;

        //Workaround for the fact that ResourceFinder initializes the network by itself. See YARP#1014
        Network network;

        ResourceFinder &resourceFinder = ResourceFinder::getResourceFinderSingleton();
        resourceFinder.configure(0, 0);

        Property configurations;
        //loading defaults from configuration file
        if (!configurations.fromConfigFile(resourceFinder.findFile(wbiConfigFile))) {
            return -1;
        }

        wbi::IDList jointList;
        //parse the file to get the joint list
        if (!WBInterface::wbdIDListFromConfigPropAndList(configurations,list,jointList)) {
            return -2;
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

        //Workaround for the fact that ResourceFinder initializes the network by itself. See YARP#1014
        Network network;
        ResourceFinder &resourceFinder = ResourceFinder::getResourceFinderSingleton();
        resourceFinder.configure(0, 0);

        //parameters needed by this block:
        // - YARP_ROBOT_NAME: needed by resource finder for resource lookup (for now it is taken by the environment)
        // - robot: robot port. If defined overrides the one specified by wbi file
        // - moduleName: local (opened) ports.
        // - wbi config file name (default: yarpWholeBodyInterface.ini): specifies the wbi config file
        // - wbi list (default ROBOT_TORQUE_CONTROL_JOINTS): specifies the WBI list.
        //            If it is a list [of style: (value1 value2 value3)] it specifies directly the list of joints to control,
        //            otherwise its value specifies the name of list present the wbi config file.

        if (m_configuration) {
            delete m_configuration;
            m_configuration = 0;
        }
        m_configuration = new Property();
        //loading defaults
        m_wbiFilePath = resourceFinder.findFile(wbiConfigFile);
        if (!m_configuration->fromConfigFile(m_wbiFilePath)) {
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

        if (!WBInterface::wbdIDListFromConfigPropAndList(*m_configuration,list,*m_robotList)) {
            if(error) error->message = "Could not load the specified WBI list (list param: " + list + " )";
            Network::fini();
            return false;
        }
        m_wbiListName = list;

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

    bool WBInterface::initialize()
    {
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

    bool WBInterface::initializeModel()
    {
        if (m_modelInitialized) {
            s_modelReferenceCount++;
            return true;
        }

        if (!m_configuration || !m_robotList) return false;

        if (m_model) {
            m_model->close();
            delete m_model;
            m_model = 0;
        }

        m_model = new yarpWbi::yarpWholeBodyModel(m_configuration->find("localName").toString().c_str(), *m_configuration);
        if (!m_model) return false;

        s_modelReferenceCount = 1;

        if (!m_model->addJoints(*m_robotList)) {
            terminateModel();
            return false;
        }

        if (!m_model->init()) {
            terminateModel();
            return false;
        }

        m_modelInitialized = true;
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

        //clean also configuration if also model has been removed
        if (!m_modelInitialized) clearConfiguration();
        m_initialized = false;
        s_referenceCount = 0;
        return result;
    }

    bool WBInterface::terminateModel()
    {
        if (s_modelReferenceCount > 1) {
            s_modelReferenceCount--;
            return true;
        }

        bool result = true;
        if (m_model) {
            result = m_model->close();
            if (result) {
                delete m_model;
                m_model = 0;
            }
        }

        //clean also configuration if also interface has been removed
        if (!m_initialized) clearConfiguration();
        m_modelInitialized = false;
        s_modelReferenceCount = 0;
        return result;
    }

    bool WBInterface::isInterfaceInitialized() const { return m_initialized; }

    const wbi::IDList* WBInterface::wbiList() const
    {
        if (m_robotList)
            return m_robotList;
        return NULL;
    }

    const std::string& WBInterface::wbiFilePath() const
    {
        return m_wbiFilePath;
    }

    const std::string& WBInterface::wbiListName() const
    {
        return m_wbiListName;
    }
}
