#ifndef WBIT_WBINTERFACE_H
#define WBIT_WBINTERFACE_H

#include <string>

namespace wbit {
    class WBInterface;
    class Error;
}

namespace wbi {
    class wholeBodyInterface;
    class IDList;
}
namespace yarp {
    namespace os {
        class Property;
    }
}

/**
 * This class holds a reference counted handle to the whole body interface object.
 * You can obtain the singleton reference to this object by calling the sharedInstance method.
 */
class wbit::WBInterface {

    // Private constructor, destructor, copy constructor and assignemnt operator
    WBInterface();
    WBInterface(const WBInterface&);
    WBInterface& operator=(const WBInterface&);
    ~WBInterface();


    wbi::wholeBodyInterface* m_interface; /*<! Reference to the interface object */
    wbi::IDList *m_robotList; /*<! Reference to the joint list object */
    yarp::os::Property *m_configuration; /*<! Reference to the configuration used to configure the interface */

    bool m_initialized; /*<! true if the interface has been initialized */
    bool m_configured; /*<! true if the interface has been configured */
    int m_dofs; /*<! dofs modelled by the interface */

    static unsigned s_referenceCount; /*<! number of blocks currently initialized */
    
public:
    /**
     * Returns the singleton instance to this object.
     *
     * This is the only way to obtain a reference to an instance of this class
     * @return the singleton instance
     */
    static wbit::WBInterface& sharedInstance();

    /**
     * Returns the constant pointer to the whole body interface object
     *
     * @return the whole body interface object
     */
    wbi::wholeBodyInterface * const interface();

    /**
     * Returns the degrees of freedom associated with the interface object
     *
     * @return degrees of freedom of the WBI object
     */
    int numberOfDoFs() const;

    /**
     * Returns the degrees of freedom for the specified configuration file and joint list
     *
     * @Note: this method also save the dofs in the internal state. It is thus possible to retrieve it
     * by calling numberOfDofs. 
     * @todo: maybe we should transform this method into const?
     * @param wbiConfigFile wbi configuration file
     * @param list          joint list to be found in the configuration file
     *
     * @return the degrees of freedom of the specified list
     */
    int dofsForConfigurationFileAndList(const std::string & wbiConfigFile, const std::string & list);

    /**
     * Configure the interface with the specified paramters.
     *
     * @Note: the interface is configured only once. Subsequent calls are ignored.
     * Call clearConfiguration() to clear the configuration.
     * @param robotName     name of the robot (The model will connect to port with this name as prefix).
     *                      empty the one specified in the configuration file is taken
     * @param localName     name of the model (ports will be open with this name)
     * @param wbiConfigFile name of the wbi configuration file. The file will be looked for by using 
     *                      yarp::os::ResourceFinder policies
     * @param list          The joint list to be configured
     * @param [out]error    Pointer to the error object to be filled in case of error. 
     *                      Check for NULL before using it
     *
     * @return true if configure is successful, false otherwise.
     */
    bool configure(const std::string &robotName,
                   const std::string & localName,
                   const std::string & wbiConfigFile,
                   const std::string & list,
                   wbit::Error *error);

    /**
     * Clear the current configuration, so that configure can be called again.
     */
    void clearConfiguration();

    /**
     * Initialize the whole body interface.
     *
     * This call has effect only the first time. Subsequent calls only increase the reference count
     * @Note: Each initialize call should be matched by a subsequent call to terminate.
     * @return: true if configure is successful, false otherwise.
     */
    bool initialize();

    /**
     * Release the resources associated with the whole body interface
     *
     * @Note: resources are released only if the call is made by the last object using the interface
     * otherwise the reference count is decreased and no resources are relased.
     * @return true if configure is successful, false otherwise.
     */
    bool terminate();

    /**
     * Returns true if the interface is initialized. False otherwise
     *
     * @return true if the interface is initialized, false otherwise
     */
    bool isInterfaceInitialized() const;

};

#endif /* end of include guard: WBIT_WBINTERFACE_H */
