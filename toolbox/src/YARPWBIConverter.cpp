#include "YARPWBIConverter.h"

#include "BlockInformation.h"
#include "Signal.h"
#include "Error.h"
#include "WBInterface.h"
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <wbi/wholeBodyInterface.h>
#include <yarp/os/ResourceFinder.h>
#include <Eigen/Core>


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
    bool YARPWBIConverter::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
    {
        //This is a check that list, dofs and files are ok
        if (!WBIBlock::configureSizeAndPorts(blockInfo, error)) {
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

        blockInfo->getScalarParameterAtIndex(wbt::WBIModelBlock::numberOfParameters() + 1).int8Data();

        bool yarpToWBI = blockInfo->getScalarParameterAtIndex(wbt::WBIModelBlock::numberOfParameters() + 1).int8Data() == 1;

        // Specify I/O
        bool success = true;
        //Input
        if (yarpToWBI) {
            if (!blockInfo->setNumberOfInputPorts(parts.size())) {
                if (error) error->message = "Failed to configure the number of input ports";
                return false;
            }

            for (unsigned i = 0 ; i < parts.size(); ++i) {
                success = success && blockInfo->setInputPortVectorSize(i, parts[i].joints.size());
                blockInfo->setInputPortType(i, PortDataTypeDouble);
            }

        } else {
            if (!blockInfo->setNumberOfInputPorts(1)) {
                if (error) error->message = "Failed to configure the number of input ports";
                return false;
            }

            success = success && blockInfo->setInputPortVectorSize(0, dofs);
            blockInfo->setInputPortType(0, PortDataTypeDouble);
        }

        if (!success) {
            if (error) error->message = "Failed to configure input ports";
            return false;
        }

        if (yarpToWBI) {
            // Output port:
            // - one port of size dofs
            if (!blockInfo->setNumberOfOuputPorts(1)) {
                if (error) error->message = "Failed to configure the number of output ports";
                return false;
            }

            success = blockInfo->setOutputPortVectorSize(0, dofs);
            blockInfo->setOutputPortType(0, PortDataTypeDouble);
        } else {
            if (!blockInfo->setNumberOfOuputPorts(parts.size())) {
                if (error) error->message = "Failed to configure the number of input ports";
                return false;
            }

            for (unsigned i = 0 ; i < parts.size(); ++i) {
                success = success && blockInfo->setOutputPortVectorSize(i, parts[i].joints.size());
                blockInfo->setOutputPortType(i, PortDataTypeDouble);
            }
        }

        if (!success) {
            if (error) error->message = "Failed to configure output ports";
            return false;
        }

        return success;
    }

    bool YARPWBIConverter::initialize(BlockInformation *blockInfo, wbt::Error *error)
    {
        using namespace wbt::hidden;
        if (!WBIModelBlock::initialize(blockInfo, error)) return false;

        m_pimpl = new YARPWBIConverterPimpl();
        if (!m_pimpl) return false;

        m_pimpl->yarpToWBI = blockInfo->getScalarParameterAtIndex(wbt::WBIModelBlock::numberOfParameters() + 1).int8Data() == 1;
        return parseConfigurationFileForPartsConfiguration(WBInterface::sharedInstance().wbiFilePath(), *WBInterface::sharedInstance().wbiList(), m_pimpl->parts);
    }

    bool YARPWBIConverter::terminate(BlockInformation *blockInfo, wbt::Error *error)
    {
        delete m_pimpl;
        m_pimpl = 0;

        return WBIModelBlock::terminate(blockInfo, error);
    }

    bool YARPWBIConverter::output(BlockInformation *blockInfo, wbt::Error *error)
    {
        if (!m_pimpl) return false;
        using namespace wbt::hidden;

        if (m_pimpl->yarpToWBI) {
            //From YARP to WBI:
            //Multiple inputs, single output

            Signal output = blockInfo->getOutputPortSignal(0);

            for (unsigned partIdx = 0; partIdx < m_pimpl->parts.size(); ++partIdx) {
                Signal partPort = blockInfo->getInputPortSignal(partIdx);

                Part part = m_pimpl->parts[partIdx];
                for (unsigned jointIdx = 0; jointIdx < part.joints.size(); ++jointIdx) {
                    Joint joint = part.joints[jointIdx];
                    output.set(joint.dofIndex, partPort.get(joint.yarpIndex).doubleData());
                }
            }

        } else {
            //From WBI to YARP
            //Single Input port, multiple outputs
            Signal inputPort = blockInfo->getInputPortSignal(0);

            for (unsigned partIdx = 0; partIdx < m_pimpl->parts.size(); ++partIdx) {
                Signal output = blockInfo->getOutputPortSignal(partIdx);

                Part part = m_pimpl->parts[partIdx];
                for (unsigned jointIdx = 0; jointIdx < part.joints.size(); ++jointIdx) {
                    Joint joint = part.joints[jointIdx];
                    output.set(joint.yarpIndex, inputPort.get(joint.dofIndex).doubleData());
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


