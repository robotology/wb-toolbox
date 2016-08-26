#include "YARPWBIConverter.h"

#include "Error.h"
#include "WBInterface.h"
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <wbi/wholeBodyInterface.h>
#include <Eigen/Core>
#include <yarp/os/ResourceFinder.h>

namespace wbt {
    namespace hidden {
        struct Joint;

        struct Part {
            std::string partName;
            std::vector<Joint> joints;
        };

        struct Joint {
            unsigned yarpIndex;
            unsigned dofIndex;
            std::string name;
        };

        /**
         * Parse the configuration file to extract parts, joints and indices
         *
         * @param filepath full path and name to the configuration file
         * @return the YARP parts representation
         */
        static bool parseConfigurationFileForPartsConfiguration(const std::string& filepath, const wbi::IDList& wbiList, std::vector<Part> &parts);

    }
}


namespace wbt {

    struct YARPWBIConverter::YARPWBIConverterPimpl {
        std::vector<wbt::hidden::Part> parts;
        bool yarpToWBI;
    };

    std::string YARPWBIConverter::ClassName = "YARPWBIConverter";

    YARPWBIConverter::YARPWBIConverter() {}

    unsigned YARPWBIConverter::numberOfParameters() { return wbt::WBIModelBlock::numberOfParameters() + 1; }
    bool YARPWBIConverter::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
    {
        //This is a check that list, dofs and files are ok
        if (!WBIBlock::configureSizeAndPorts(S, error)) {
            return false;
        }

        //This specify the output port size
        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();

        using namespace wbt::hidden;
        //Parse the yarpwbi file
        //Need to retrieve both parameters, i.e. wbiFile and list
        //The following is to avoid to configure the WBInterface during compilation
        //(why I don't know :D )
        std::vector<Part> parts;

        //Workaround for the fact that ResourceFinder initializes the network by itself. See YARP#1014
        yarp::os::Network network;
        yarp::os::ResourceFinder resourceFinder = yarp::os::ResourceFinder::getResourceFinderSingleton();

        std::string configFilePath = resourceFinder.findFile(m_wbiConfigurationFileName);

        yarp::os::Property configurations;
        //loading defaults from configuration file
        if (!configurations.fromConfigFile(configFilePath)) {
            if (error) error->message = "Failed to parse WBI configuration file";
            return false;
        }

        wbi::IDList jointList;
        //parse the file to get the joint list
        if (!WBInterface::wbdIDListFromConfigPropAndList(configurations, m_wbiListName, jointList)) {
            if (error) error->message = "Failed to parse Joint list";
            return false;
        }

        if (!parseConfigurationFileForPartsConfiguration(configFilePath, jointList, parts)) {
            return false;
        }

        int yarpToWBIOption = static_cast<int>(mxGetScalar(ssGetSFcnParam(S, wbt::WBIModelBlock::numberOfParameters() + 1)));
        bool yarpToWBI = yarpToWBIOption == 1;

        // Specify I/O
        bool success = true;
        //Input
        if (yarpToWBI) {
            if (!ssSetNumInputPorts (S, parts.size())) {
                if (error) error->message = "Failed to configure the number of input ports";
                return false;
            }

            for (unsigned i = 0 ; i < parts.size(); ++i) {
                success = success && ssSetInputPortVectorDimension(S, i, parts[i].joints.size());
                ssSetInputPortDataType (S, i, SS_DOUBLE);
                ssSetInputPortDirectFeedThrough (S, i, 1);
            }

        } else {
            if (!ssSetNumInputPorts (S, 1)) {
                if (error) error->message = "Failed to configure the number of input ports";
                return false;
            }

            success = success && ssSetInputPortVectorDimension(S, 0, dofs);
            ssSetInputPortDataType (S, 0, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough (S, 0, 1);

        }

        if (!success) {
            if (error) error->message = "Failed to configure input ports";
            return false;
        }

        if (yarpToWBI) {
            // Output port:
            // - one port of size dofs
            if (!ssSetNumOutputPorts (S, 1)) {
                if (error) error->message = "Failed to configure the number of output ports";
                return false;
            }

            success = ssSetOutputPortVectorDimension(S, 0, dofs);
            ssSetOutputPortDataType (S, 0, SS_DOUBLE);
        } else {
            if (!ssSetNumOutputPorts (S, parts.size())) {
                if (error) error->message = "Failed to configure the number of input ports";
                return false;
            }

            for (unsigned i = 0 ; i < parts.size(); ++i) {
                success = success && ssSetOutputPortVectorDimension(S, i, parts[i].joints.size());
                ssSetOutputPortDataType (S, i, SS_DOUBLE);
            }
        }

        if (!success) {
            if (error) error->message = "Failed to configure output ports";
            return false;
        }

        return success;
    }

    bool YARPWBIConverter::initialize(SimStruct *S, wbt::Error *error)
    {
        using namespace wbt::hidden;
        if (!WBIModelBlock::initialize(S, error)) return false;

        m_pimpl = new YARPWBIConverterPimpl();
        if (!m_pimpl) return false;

        int yarpToWBIOption = static_cast<int>(mxGetScalar(ssGetSFcnParam(S, wbt::WBIModelBlock::numberOfParameters() + 1)));
        m_pimpl->yarpToWBI = yarpToWBIOption == 1;
        return parseConfigurationFileForPartsConfiguration(WBInterface::sharedInstance().wbiFilePath(), *WBInterface::sharedInstance().wbiList(), m_pimpl->parts);
    }

    bool YARPWBIConverter::terminate(SimStruct *S, wbt::Error *error)
    {
        if (m_pimpl) {
            delete m_pimpl;
            m_pimpl = 0;
        }
        return WBIModelBlock::terminate(S, error);
    }

    bool YARPWBIConverter::output(SimStruct *S, wbt::Error */*error*/)
    {
        if (!m_pimpl) return false;
        using namespace wbt::hidden;

        if (m_pimpl->yarpToWBI) {
            //From YARP to WBI:
            //Multiple inputs, single output
            real_T *output = ssGetOutputPortRealSignal(S, 0);

            for (unsigned partIdx = 0; partIdx < m_pimpl->parts.size(); ++partIdx) {
                InputRealPtrsType partPort = ssGetInputPortRealSignalPtrs(S, partIdx);

                Part part = m_pimpl->parts[partIdx];
                for (unsigned jointIdx = 0; jointIdx < part.joints.size(); ++jointIdx) {
                    Joint joint = part.joints[jointIdx];
                    output[joint.dofIndex] = *partPort[joint.yarpIndex];
                }
            }

        } else {
            //From WBI to YARP
            //Single Input port, multiple outputs
            InputRealPtrsType inputPort = ssGetInputPortRealSignalPtrs(S, 0);

            for (unsigned partIdx = 0; partIdx < m_pimpl->parts.size(); ++partIdx) {
                real_T *output = ssGetOutputPortRealSignal(S, partIdx);

                Part part = m_pimpl->parts[partIdx];
                for (unsigned jointIdx = 0; jointIdx < part.joints.size(); ++jointIdx) {
                    Joint joint = part.joints[jointIdx];
                    output[joint.yarpIndex] = *inputPort[joint.dofIndex];
                }

            }
        }

        return true;
    }

    namespace hidden {

        bool parseConfigurationFileForPartsConfiguration(const std::string& filepath, const wbi::IDList& wbiList, std::vector<Part> &parts)
        {
            parts.clear();
            yarp::os::Property configurationFile;
            configurationFile.fromConfigFile(filepath);
            //retrieve joints/yarp mapping
            yarp::os::Bottle &mapping = configurationFile.findGroup("WBI_YARP_JOINTS");
            if (mapping.isNull() || mapping.size() <= 0) {
                return false;
            }

            //for every joint in the list
            for (unsigned i = 0; i < wbiList.size(); ++i) {
                const wbi::ID &id = wbiList.at(i);
                //look in the configuration file for that key
                yarp::os::Value &jointList = mapping.find(id.toString());

                if (!jointList.isNull() && jointList.isList() && jointList.asList()->size() == 2) {
                    yarp::os::Bottle *list = jointList.asList();
                    Joint joint;
                    std::string partName = list->get(0).asString();
                    joint.yarpIndex = list->get(1).asInt();
                    joint.dofIndex = i;
                    joint.name = id.toString();

                    std::vector<Part>::iterator found = parts.end();
                    //look for the part in the vector.. a map would be nicer
                    for (std::vector<Part>::iterator part = parts.begin();
                         part != parts.end(); ++part) {
                        if (part->partName == partName) {
                            found = part;
                            break;
                        }
                    }

                    if (found != parts.end()) {
                        found->joints.push_back(joint);
                    } else {
                        Part newPart;
                        newPart.partName = partName;
                        newPart.joints.push_back(joint);
                        parts.push_back(newPart);
                    }
                }
            }
            return true;
        }
    }
}


