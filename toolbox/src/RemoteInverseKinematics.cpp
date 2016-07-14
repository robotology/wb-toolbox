#include "RemoteInverseKinematics.h"

#include "Error.h"
#include <Eigen/Core>
#include <yarp/math/Math.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/RpcClient.h>
#include <cmath>

//use http://wiki.icub.org/brain/group__iKinSlv.html

namespace wbt {

    struct RemoteInverseKinematics::RemoteInverseKinematicsPimpl {
        bool m_firstTime;
        //input buffers
        double *m_configuration;

        std::string m_solverInputPortName;
        std::string m_solverOutputPortName;

        yarp::os::BufferedPort<yarp::os::Bottle> m_solverInputPort;
        yarp::os::BufferedPort<yarp::os::Bottle> m_solverOutputPort;

        double m_lastToken;

        yarp::sig::Matrix m_desiredPoseAsMatrix;
        yarp::sig::Vector m_desiredPoseAsAngleAxis;
        yarp::sig::Vector m_desiredAngleAxisOrientation;

        RemoteInverseKinematicsPimpl()
        : m_firstTime(true)
        , m_configuration(0) {}

        ~RemoteInverseKinematicsPimpl()
        {
            if (m_configuration) {
                delete [] m_configuration;
                m_configuration = 0;
            }
        }

    };

    std::string RemoteInverseKinematics::ClassName = "RemoteInverseKinematics";

    RemoteInverseKinematics::RemoteInverseKinematics()
    : m_piml(0) {}

    unsigned RemoteInverseKinematics::numberOfParameters()
    {
        return 3;
        //Parameter 1: Yarp Name of the solver
        //Parameter 2: DOFs of the chain
        //Parameter 3: type of solution (Int)
    }

    unsigned RemoteInverseKinematics::numberOfDiscreteStates() { return 1; } //fake state to force the call of the output

    bool RemoteInverseKinematics::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
    {
        int dofs = mxGetScalar(ssGetSFcnParam(S, 2));

        // Specify I/O
        // Input ports:
        // - 4x4 matrix (homogenous transformation for the desired end effector pose w.r.t. world)

        if (!ssSetNumInputPorts (S, 2)) {
            if (error) error->message = "Failed to configure the number of input ports";
            return false;
        }
        bool success = true;
        success = success && ssSetInputPortMatrixDimensions(S,  0, 4, 4); //desired pose
        success = success && ssSetInputPortVectorDimension(S, 1, dofs); //joint configuration

        ssSetInputPortDataType (S, 0, SS_DOUBLE);
        ssSetInputPortDataType (S, 1, SS_DOUBLE);

        ssSetInputPortDirectFeedThrough (S, 0, 1);
        ssSetInputPortDirectFeedThrough (S, 1, 1);

        if (!success) {
            if (error) error->message = "Failed to configure input ports";
            return false;
        }

        // Output port:
        // - DoFs desired joints configuration
        if (!ssSetNumOutputPorts (S, 1)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        success = ssSetOutputPortVectorDimension(S, 0, dofs);
        ssSetOutputPortDataType (S, 0, SS_DOUBLE);

        return success;
    }

    bool RemoteInverseKinematics::initialize(SimStruct *S, wbt::Error *error)
    {
        using namespace yarp::os;

        m_piml = new RemoteInverseKinematicsPimpl();
        if (!m_piml) return false;

        int parentParameters = 1;
        std::string solverName;
        if (!Block::readStringParameterAtIndex(S, parentParameters, solverName)) {
            if (error) error->message = "Cannot retrieve string from solver yarp name";
            return false;
        }
        parentParameters++;
        int dofs = mxGetScalar(ssGetSFcnParam(S,parentParameters));

        parentParameters++;

        int optOption = mxGetScalar(ssGetSFcnParam(S,parentParameters));

        m_piml->m_configuration = new double[dofs];
        if (!m_piml->m_configuration) return false;

        //Open ports and connect
        std::string rpcPortName = solverName;
        m_piml->m_solverInputPortName = solverName;
        m_piml->m_solverOutputPortName = solverName;
        if (solverName[solverName.length() - 1] != '/') {
            m_piml->m_solverInputPortName.append("/");
            m_piml->m_solverOutputPortName.append("/");
            rpcPortName.append("/");
        }
        m_piml->m_solverInputPortName.append("in");
        m_piml->m_solverOutputPortName.append("out");
        rpcPortName.append("rpc");

        bool result = m_piml->m_solverInputPort.open("...");
        result = result && m_piml->m_solverOutputPort.open("...");

        //also rpc, e.g. pose
        yarp::os::RpcClient rpc;
        if (rpc.open(rpcPortName)) {
            yarp::os::Bottle response;
            yarp::os::Bottle request;
            request.addString("set pose");
            if (optOption == 1)
                request.addString("full");
            else
                request.addString("xyz");

            rpc.write(request, response);
            rpc.close();
        }

        result = result && yarp::os::Network::connect(m_piml->m_solverInputPort.getName(), m_piml->m_solverInputPortName);
        result = result && yarp::os::Network::connect(m_piml->m_solverOutputPortName, m_piml->m_solverOutputPort.getName());


        m_piml->m_desiredPoseAsMatrix.resize(4, 4);
        m_piml->m_desiredPoseAsAngleAxis.resize(7, 0.0);
        m_piml->m_desiredAngleAxisOrientation.resize(4, 0.0);

        m_piml->m_lastToken = -1;

        m_piml->m_firstTime = true;

        if (!result && error) {
            error->message = "Could not establish connection with iKin solver " + solverName;
        }
        return result;
    }

    bool RemoteInverseKinematics::terminate(SimStruct *S, wbt::Error *error)
    {
        bool result = true;
        if (m_piml) {
            result = yarp::os::Network::disconnect(m_piml->m_solverInputPort.getName(), m_piml->m_solverInputPortName);
            result = result && yarp::os::Network::disconnect(m_piml->m_solverOutputPortName, m_piml->m_solverOutputPort.getName());

            m_piml->m_solverInputPort.close();
            m_piml->m_solverOutputPort.close();


            delete m_piml;
            m_piml = 0;
        }
        return result;
    }

    bool RemoteInverseKinematics::output(SimStruct *S, wbt::Error */*error*/)
    {
        //get input
        if (m_piml->m_firstTime) {
            InputRealPtrsType configuration = ssGetInputPortRealSignalPtrs(S, 1);
            for (unsigned i = 0; i < ssGetInputPortWidth(S, 1); ++i) {
                m_piml->m_configuration[i] = *configuration[i];
            }
        }

        if (m_piml->m_lastToken == -1) {
            //write request to solver
            m_piml->m_lastToken = yarp::os::Time::now();

            //to do the request we have to read the inputs..
            InputRealPtrsType desiredPoseRaw = ssGetInputPortRealSignalPtrs(S, 0);

            double desiredPoseColMajorRaw[16];

            for (unsigned i = 0; i < ssGetInputPortWidth(S, 0); ++i) {
                desiredPoseColMajorRaw[i] = *desiredPoseRaw[i];
            }

            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor> > desiredPoseColMajor(desiredPoseColMajorRaw);
            //Convert desiredPoseColMajor to yarp matrix
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    m_piml->m_desiredPoseAsMatrix(row, col) = desiredPoseColMajor(row, col);
                }
            }

            //To vector
            m_piml->m_desiredPoseAsAngleAxis(0) = m_piml->m_desiredPoseAsMatrix(0, 3);
            m_piml->m_desiredPoseAsAngleAxis(1) = m_piml->m_desiredPoseAsMatrix(1, 3);
            m_piml->m_desiredPoseAsAngleAxis(2) = m_piml->m_desiredPoseAsMatrix(2, 3);

            m_piml->m_desiredAngleAxisOrientation = yarp::math::dcm2axis(m_piml->m_desiredPoseAsMatrix);
            m_piml->m_desiredPoseAsAngleAxis.setSubvector(3, m_piml->m_desiredAngleAxisOrientation);

            yarp::os::Bottle &inputCommand = m_piml->m_solverInputPort.prepare();

            inputCommand.clear();
            yarp::os::Bottle &pose = inputCommand.addList();
            pose.addString("xd");
            pose.addList().read(m_piml->m_desiredPoseAsAngleAxis);

            //how to add token?
            //how to add initial solution??
            m_piml->m_solverInputPort.write();

        }
        if (m_piml->m_lastToken > 0) {
            //wait for reply

            //non blocking
            yarp::os::Bottle *solverOutput = m_piml->m_solverOutputPort.read(false);
            if (solverOutput) {
                if (m_piml->m_firstTime) m_piml->m_firstTime = false;

                m_piml->m_lastToken = -1;
                yarp::os::Value &joints = solverOutput->find("q");
                if (!joints.isNull() && joints.isList()) {

                    yarp::os::Bottle *jointList = joints.asList();
                    if (jointList->size() == ssGetOutputPortWidth(S, 0)) {
                        for (int i = 0; i < jointList->size(); ++i) {
                            m_piml->m_configuration[i] = jointList->get(i).asDouble() * M_PI / 180.0;
                        }
                    }
                }
            }
        }

        real_T *output = ssGetOutputPortRealSignal(S, 0);
        for (int i = 0; i < ssGetOutputPortWidth(S, 0); ++i) {
            output[i] = m_piml->m_configuration[i];
        }

        return true;

    }

}
