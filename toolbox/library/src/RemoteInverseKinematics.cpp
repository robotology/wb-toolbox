#include "RemoteInverseKinematics.h"
#include "BlockInformation.h"
#include "Error.h"
#include "Signal.h"

#include <Eigen/Core>
#include <yarp/math/Math.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

#include <cmath>
#include <condition_variable>
#include <mutex>

// use http://wiki.icub.org/brain/group__iKinSlv.html

namespace wbt {
    struct RemoteInverseKinematics::RemoteInverseKinematicsPimpl
    {

        class SolverRPCReader : public yarp::os::Thread
        {
            RemoteInverseKinematicsPimpl& pimpl;

            bool requestPending;
            bool newRequest;

            std::mutex mutex;
            std::condition_variable requestCondition;

            yarp::sig::Vector xd;
            yarp::sig::Vector q0;

            yarp::os::Bottle request;
            yarp::os::Bottle response;

        public:
            SolverRPCReader(RemoteInverseKinematicsPimpl& pimpl)
                : pimpl(pimpl)
                , requestPending(false)
                , newRequest(false)
            {
                xd.resize(7);
                q0.resize(pimpl.m_configuration.size());
            }

            virtual void onStop()
            {
                std::unique_lock<std::mutex> guard(mutex);
                requestCondition.notify_all();
            }

            virtual void run()
            {
                while (true) {
                    {
                        std::unique_lock<std::mutex> guard(mutex);
                        // while !newRequest
                        while (!newRequest) {
                            if (isStopping()) {
                                break;
                            }
                            requestCondition.wait(guard);
                        }
                        if (isStopping()){
                            break;}

                        requestPending = true;
                        newRequest = false;
                    }

                    request.clear();
                    response.clear();

                    request.addString("ask");
                    yarp::os::Bottle& pose = request.addList();
                    pose.addString("xd");
                    pose.addList().read(xd);
                    yarp::os::Bottle& qInitial = request.addList();
                    qInitial.addString("q");
                    qInitial.addList().read(q0);

                    pimpl.m_rpc.write(request, response);

                    yarp::os::Value& joints = response.find("q");
                    if (!joints.isNull() && joints.isList()) {
                        yarp::os::Bottle* jointList = joints.asList();
                        if (jointList->size() >= q0.size()) {
                            std::lock_guard<std::mutex> outputLock(pimpl.m_outputMutex);
                            for (int i = 0; i < jointList->size(); ++i) {
                                pimpl.m_configuration[i] =
                                    jointList->get(i).asDouble() * M_PI / 180.0;
                            }
                        }
                    }

                    {
                        std::lock_guard<std::mutex> guard(mutex);
                        requestPending = false;
                    }
                }
            }

            void addRequest(const yarp::sig::Vector& xd, const yarp::sig::Vector& q0)
            {
                std::unique_lock<std::mutex> guard(mutex);
                if (requestPending)
                    return;
                this->xd = xd;
                this->q0 = q0;
                newRequest = true;
                requestCondition.notify_one();
            }
        };

        bool m_firstTime;
        // input buffers
        std::mutex m_outputMutex;
        yarp::sig::Vector m_configuration;
        yarp::os::RpcClient m_rpc;
        yarp::os::Thread* m_solverThread;

        std::string m_solverRPCPortName;

        yarp::sig::Matrix m_desiredPoseAsMatrix;
        yarp::sig::Vector m_desiredPoseAsAngleAxis;
        yarp::sig::Vector m_desiredAngleAxisOrientation;

        RemoteInverseKinematicsPimpl()
            : m_firstTime(true)
        {}

        ~RemoteInverseKinematicsPimpl() {}
    };

    std::string RemoteInverseKinematics::ClassName = "RemoteInverseKinematics";

    RemoteInverseKinematics::RemoteInverseKinematics()
        : m_piml(0)
    {}

    unsigned RemoteInverseKinematics::numberOfParameters()
    {
        return 3;
        // Parameter 1: Yarp Name of the solver
        // Parameter 2: DOFs of the chain
        // Parameter 3: type of solution (Int)
    }

    unsigned RemoteInverseKinematics::numberOfDiscreteStates()
    {
        return 1;
    } // fake state to force the call of the output

    bool RemoteInverseKinematics::configureSizeAndPorts(BlockInformation* blockInfo,
                                                        wbt::Error* error)
    {
        int dofs = blockInfo->getScalarParameterAtIndex(2).int32Data();

        // Specify I/O
        // Input ports:
        // - 4x4 matrix (homogenous transformation for the desired end effector pose w.r.t. world)

        if (!blockInfo->setNumberOfInputPorts(2)) {
            if (error) {
                error->message = "Failed to configure the number of input ports";
            }
            return false;
        }
        bool success = true;
        success = success && blockInfo->setInputPortMatrixSize(0, 4, 4); // desired pose
        success = success && blockInfo->setInputPortVectorSize(1, dofs); // joint configuration

        success = success && blockInfo->setInputPortType(0, PortDataTypeDouble);
        success = success && blockInfo->setInputPortType(1, PortDataTypeDouble);

        if (!success) {
            if (error) {
                error->message = "Failed to configure input ports";
            }
            return false;
        }

        // Output port:
        // - DoFs desired joints configuration
        if (!blockInfo->setNumberOfOuputPorts(1)) {
            if (error) {
                error->message = "Failed to configure the number of output ports"
            };
            return false;
        }

        success = blockInfo->setOutputPortVectorSize(0, dofs);
        success = success && blockInfo->setOutputPortType(0, PortDataTypeDouble);

        return success;
    }

    bool RemoteInverseKinematics::initialize(BlockInformation* blockInfo, wbt::Error* error)
    {
        using namespace yarp::os;

        Network::init();

        if (!Network::initialized() || !Network::checkNetwork(5.0)) {
            if (error) {
                error->message = "YARP server wasn't found active!! \n";
            }
            return false;
        }

        m_piml = new RemoteInverseKinematicsPimpl();
        if (!m_piml)
            return false;

        int parentParameters = 1;
        std::string solverName;
        if (!blockInfo->getStringParameterAtIndex(parentParameters, solverName)) {
            if (error) {
                error->message = "Cannot retrieve string from solver yarp name";
            }
            return false;
        }
        parentParameters++;
        int dofs = blockInfo->getScalarParameterAtIndex(2).int32Data();

        parentParameters++;

        int optOption = blockInfo->getScalarParameterAtIndex(2).int8Data();

        m_piml->m_configuration.resize(dofs);

        // Open ports and connect
        m_piml->m_solverRPCPortName = solverName;
        if (solverName[solverName.length() - 1] != '/') {
            m_piml->m_solverRPCPortName.append("/");
        }
        m_piml->m_solverRPCPortName.append("rpc");

        // also rpc, e.g. pose
        if (m_piml->m_rpc.open("...")
            && yarp::os::Network::connect(m_piml->m_rpc.getName(), m_piml->m_solverRPCPortName)) {
            yarp::os::Bottle response;
            yarp::os::Bottle request;
            request.addString("set pose");
            if (optOption == 1)
                request.addString("full");
            else
                request.addString("xyz");

            m_piml->m_rpc.write(request, response);
        }

        m_piml->m_desiredPoseAsMatrix.resize(4, 4);
        m_piml->m_desiredPoseAsAngleAxis.resize(7, 0.0);
        m_piml->m_desiredAngleAxisOrientation.resize(4, 0.0);

        m_piml->m_firstTime = true;

        m_piml->m_solverThread = new RemoteInverseKinematicsPimpl::SolverRPCReader(*m_piml);
        return m_piml->m_solverThread && m_piml->m_solverThread->start();
    }

    bool RemoteInverseKinematics::terminate(BlockInformation* blockInfo, wbt::Error* error)
    {
        bool result = true;
        if (m_piml) {
            if (m_piml->m_solverThread) {
                m_piml->m_solverThread->stop();
                delete m_piml->m_solverThread;
                m_piml->m_solverThread = 0;
            }
            yarp::os::Network::disconnect(m_piml->m_rpc.getName(), m_piml->m_solverRPCPortName);
            m_piml->m_rpc.close();

            delete m_piml;
            m_piml = 0;
        }
        yarp::os::Network::fini();
        return result;
    }

    bool RemoteInverseKinematics::output(BlockInformation* blockInfo, wbt::Error* /*error*/)
    {
        // get input
        if (m_piml->m_firstTime) {
            m_piml->m_firstTime = false;
            Signal configuration = blockInfo->getInputPortSignal(1);

            std::lock_guard<std::mutex> outputLock(m_piml->m_outputMutex);
            for (unsigned i = 0; i < blockInfo->getInputPortWidth(1); ++i) {
                m_piml->m_configuration[i] = configuration.get(i).doubleData();
            }
        }

        // to do the request we have to read the inputs..
        Signal desiredPoseRaw = blockInfo->getInputPortSignal(0);

        double desiredPoseColMajorRaw[16];

        for (unsigned i = 0; i < blockInfo->getInputPortWidth(0); ++i) {
            desiredPoseColMajorRaw[i] = desiredPoseRaw.get(i).doubleData();
        }

        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> desiredPoseColMajor(
            desiredPoseColMajorRaw);
        // Convert desiredPoseColMajor to yarp matrix
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                m_piml->m_desiredPoseAsMatrix(row, col) = desiredPoseColMajor(row, col);
            }
        }

        // To vector
        m_piml->m_desiredPoseAsAngleAxis(0) = m_piml->m_desiredPoseAsMatrix(0, 3);
        m_piml->m_desiredPoseAsAngleAxis(1) = m_piml->m_desiredPoseAsMatrix(1, 3);
        m_piml->m_desiredPoseAsAngleAxis(2) = m_piml->m_desiredPoseAsMatrix(2, 3);

        m_piml->m_desiredAngleAxisOrientation = yarp::math::dcm2axis(m_piml->m_desiredPoseAsMatrix);
        m_piml->m_desiredPoseAsAngleAxis.setSubvector(3, m_piml->m_desiredAngleAxisOrientation);

        ((RemoteInverseKinematicsPimpl::SolverRPCReader*) m_piml->m_solverThread)
            ->addRequest(m_piml->m_desiredPoseAsAngleAxis, m_piml->m_configuration);

        Signal output = blockInfo->getOutputPortSignal(0);
        std::lock_guard<std::mutex> outputLock(m_piml->m_outputMutex);
        output.setBuffer(m_piml->m_configuration.data(), blockInfo->getOutputPortWidth(0));

        return true;
    }

} // namespace wbt
