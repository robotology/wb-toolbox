#include "SimulatorSynchronizer.h"
#include "Log.h"
#include "BlockInformation.h"
#include "thrift/ClockServer.h"

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <cmath>

using namespace wbt;

struct SimulatorSynchronizer::RPCData
{
    struct {
        std::string clientPortName;
        std::string serverPortName;

        unsigned numberOfSteps;
    } configuration;

    yarp::os::Port clientPort;
    gazebo::ClockServer clockServer;
};

const std::string SimulatorSynchronizer::ClassName = "SimulatorSynchronizer";

const unsigned SimulatorSynchronizer::PARAM_PERIOD     = 1;
const unsigned SimulatorSynchronizer::PARAM_GZCLK_PORT = 2;
const unsigned SimulatorSynchronizer::PARAM_RPC_PORT   = 3;

SimulatorSynchronizer::SimulatorSynchronizer()
: m_period(0.01)
, m_firstRun(true)
, m_rpcData(0)
{}

unsigned SimulatorSynchronizer::numberOfParameters() { return 3; }

std::vector<std::string> SimulatorSynchronizer::additionalBlockOptions()
{
    return std::vector<std::string>(1, wbt::BlockOptionPrioritizeOrder);
}

bool SimulatorSynchronizer::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // INPUTS
    // ======
    //
    // No inputs
    //

    if (!blockInfo->setNumberOfInputPorts(0)) {
        Log::getSingleton().error("Failed to set input port number to 0.");
        return false;
    }

    // OUTPUT
    // ======
    //
    // No outputs
    //

    if (!blockInfo->setNumberOfOutputPorts(0)) {
        Log::getSingleton().error("Failed to set output port number.");
        return false;
    }

    return true;
}

bool SimulatorSynchronizer::initialize(const BlockInformation* blockInfo)
{
    std::string serverPortName;
    std::string clientPortName;

    bool ok = true;
    ok = ok & blockInfo->getScalarParameterAtIndex(PARAM_PERIOD, m_period);
    ok = ok & blockInfo->getStringParameterAtIndex(PARAM_GZCLK_PORT, serverPortName);
    ok = ok & blockInfo->getStringParameterAtIndex(PARAM_RPC_PORT, clientPortName);

    if (!ok) {
        Log::getSingleton().error("Error reading RPC parameters.");
        return false;
    }

    yarp::os::Network::init();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork()) {
        Log::getSingleton().error("Error initializing Yarp network.");
        return false;
    }

    m_rpcData = new struct RPCData();
    if (!m_rpcData) {
        Log::getSingleton().error("Error creating RPC data structure.");
        return false;
    }

    m_rpcData->configuration.clientPortName = clientPortName;
    m_rpcData->configuration.serverPortName = serverPortName;

    m_firstRun = true;

    return true;
}

bool SimulatorSynchronizer::terminate(const BlockInformation* /*S*/)
{
    if (m_rpcData) {
        if (m_rpcData->clientPort.isOpen()) {
            m_rpcData->clockServer.continueSimulation();
            if (!yarp::os::Network::disconnect(m_rpcData->configuration.clientPortName,
                                               m_rpcData->configuration.serverPortName)) {
                Log::getSingleton().error("Error disconnecting from simulator clock server.");
            }
            m_rpcData->clientPort.close();
        }
        delete m_rpcData;
        m_rpcData = nullptr;
    }
    yarp::os::Network::fini();
    return true;
}

bool SimulatorSynchronizer::output(const BlockInformation* /*S*/)
{
    if (m_firstRun) {
        m_firstRun = false;

        if (!m_rpcData->clientPort.open(m_rpcData->configuration.clientPortName) ||
            !yarp::os::Network::connect(m_rpcData->configuration.clientPortName,
                                        m_rpcData->configuration.serverPortName)) {
            Log::getSingleton().error("Error connecting to simulator clock server.");
            return false;
        }

        m_rpcData->clockServer.yarp().attachAsClient(m_rpcData->clientPort);

        double stepSize = m_rpcData->clockServer.getStepSize();
        m_rpcData->configuration.numberOfSteps = static_cast<unsigned>(m_period / stepSize);
        m_rpcData->clockServer.pauseSimulation();
    }
    m_rpcData->clockServer.stepSimulationAndWait(m_rpcData->configuration.numberOfSteps);

    return true;
}
