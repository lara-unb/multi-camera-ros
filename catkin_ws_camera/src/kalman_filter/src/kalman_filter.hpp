#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#define POSE_VECTOR_SIZE 7
#define LINEAR_VECTOR_SIZE 3
#define ANGULAR_VECTOR_SIZE 4
#define HIGH_COVARIANCE_PRESET 0.1
#define PROCESS_LINEAR_NOISE_COVARIANCE 0.1
#define PROCESS_ANGULAR_NOISE_COVARIANCE 0.001

//	TIME UPDATE EQUATIONS
//	X[k] = A * X[k-1] + w[k1]							// Predict a state based on the process stochastic model
//	Y[k] = H * X[k] + v[k]								// Predict output based on the measurement stochastic model
//	P[k] = A * P[k-1] * A^{T} + Q						// Predict error covariance
//
//	p(w) ~ N(0,Q) // Normal distribution with mean of 0 and covariance of Q
//	p(v) ~ N(0,R) // Normal distribution with mean of 0 and covariance of R
//
//	MEASUREMENTS UPDATE EQUATIONS
//	K[k] = P[k] * H^{T} * ((H * P[k] * H^{T} + R)^{-1})	// Calculating Kalman gain
//	X[k] = X[k] + K[k] * (Y[k] - H * X[k])				// after measuring Y[k], calculating a posteriori state estimate
//	P[k] = (I - K[k] * H) * P[k]						// Calculate a posteriori error covariance estimate

class filter{

	public:
		Eigen::VectorXd X; 		// State vector
		Eigen::VectorXd X0; 	// Initial state vector
		Eigen::MatrixXd P; 		// Covariance matrix
		Eigen::MatrixXd P0; 	// Initial covariance matrix
		Eigen::MatrixXd A;  	// State transition matrix, presumably constant and equal identity
		Eigen::MatrixXd H;  	// Observation/measurement matrix, presumably constant and equal identity
		Eigen::MatrixXd Q;  	// Process noise covariance
		Eigen::MatrixXd Q0;		// Reference process noise covariance
		Eigen::MatrixXd R;  	// Measurement noise covariance
		Eigen::MatrixXd R0;		// Reference measurement noise covariance
		Eigen::MatrixXd K;		// Kalman filter gain
		Eigen::VectorXd Y_pred; // Predicted output
		Eigen::VectorXd Y_cor;	// Corrected output
		Eigen::VectorXd I_cor;	// Corrected measurement innovation/residual
		Eigen::VectorXd I_pred; // Predicterd measurement innovation/residual

		// Function prototypes for non-linear models and their Jacobians
		Eigen::VectorXd stateTransition(Eigen::VectorXd& state);
		Eigen::MatrixXd stateTransitionJacobian(Eigen::VectorXd& state);
		Eigen::VectorXd measurementModel(Eigen::VectorXd& state);
		Eigen::MatrixXd measurementJacobian(Eigen::VectorXd& state);

		void set_dimensions(int dim){
			X.resize(dim);
			X0.resize(dim);
			P.resize(dim, dim);
			P.setZero();
			P0.resize(dim, dim);
			P0.setIdentity() * HIGH_COVARIANCE_PRESET;
			A.setIdentity(dim, dim);
			H.setIdentity(dim, dim);
			Q.resize(dim, dim);
			Q0.resize(POSE_VECTOR_SIZE, POSE_VECTOR_SIZE);
			Q0.setZero();

			for(int i = 0; i < Q0.rows(); i++){
				if(i % POSE_VECTOR_SIZE < 3){
					Q0(i,i) = PROCESS_LINEAR_NOISE_COVARIANCE;
				}else{
					Q0(i,i) = PROCESS_ANGULAR_NOISE_COVARIANCE;
				}
			}

			R.resize(dim, dim);
			R0 = Eigen::MatrixXd::Identity(POSE_VECTOR_SIZE, POSE_VECTOR_SIZE);
		}

		void resetWorld(int worldAddr){
			P.block(worldAddr, worldAddr, POSE_VECTOR_SIZE, POSE_VECTOR_SIZE) = Eigen::MatrixXd::Identity(POSE_VECTOR_SIZE, POSE_VECTOR_SIZE);
			X.segment(worldAddr, POSE_VECTOR_SIZE).setZero();
		}

		void insertState(Eigen::VectorXd state){
			// Regarding the state vector
			int size_difference = state.size() - X.size(); // Number of new elements to be added = new poses * POSE_VECTOR_SIZE
			X.conservativeResize(X.rows() + size_difference);
			X.tail(state.size()) = state;

			// Regarding the Covariance matrix
			Eigen::MatrixXd auxMatrix(P.rows() + size_difference, P.rows() + size_difference);
			auxMatrix.setZero();
			auxMatrix.topLeftCorner(P.rows(), P.rows()) = P;
			auxMatrix.bottomRightCorner(size_difference, size_difference) = Eigen::MatrixXd::Identity(size_difference, size_difference) * HIGH_COVARIANCE_PRESET;
			P = auxMatrix;

			ROS_INFO("P-OLD:");
			for(int i = 0;i < P.cols();i++){
				for(int j = 0;j < P.rows();j++){
					ROS_INFO_STREAM(P(i,j));
				}
			}

			// Regarding the observation matrix
			A = Eigen::MatrixXd::Identity(A.rows() + size_difference, A.cols() + size_difference);

			// Regarding the observation matrix
			H = Eigen::MatrixXd::Identity(H.rows() + size_difference, H.cols() + size_difference);

			// Regarding the process noise covariance matrix
			Q.resize(Q.rows() + size_difference, Q.cols() + size_difference);
			Q.setZero();

			for(int i = 0; i < Q.rows(); i++){
				if(i % POSE_VECTOR_SIZE < 3){
					Q(i,i) = PROCESS_LINEAR_NOISE_COVARIANCE;
				}else{
					Q(i,i) = PROCESS_ANGULAR_NOISE_COVARIANCE;
				}
			}

			// Regarding the measurement noise covariance matrix
			R = Eigen::MatrixXd::Identity(R.rows() + size_difference, R.cols() + size_difference);

			if(X.rows() == POSE_VECTOR_SIZE){
				X.setZero();
				P.setIdentity();
				Q = Q0;
			}

			ROS_INFO("Q:");
			for (int i = 0; i < Q.rows(); ++i) {
				std::string row_str = "[ ";
				for (int j = 0; j < Q.cols(); ++j) {
					row_str += std::to_string(Q(i, j)) + " ";
				}
				row_str += "]";
				ROS_INFO_STREAM(row_str);
			}
		}

		void reset(){
			X = X0;
			P = P0;
			Q = Q0;
			R = R0;
		}

		void predict(){
			// Predicst state vector
			X = A * X;

			for(int i=0;i < X.size();i++){
				ROS_INFO("old x%d = %f",i, X[i]);
			}

			// Predicts Covariance error matrix
			P = A * P * A.transpose() + Q;

			ROS_INFO("P-new:");
			for (int i = 0; i < P.rows(); ++i) {
				std::string row_str = "[ ";
				for (int j = 0; j < P.cols(); ++j) {
					row_str += std::to_string(P(i, j)) + " ";
				}
				row_str += "]";
				ROS_INFO_STREAM(row_str);
			}
		}

		void correct(Eigen::VectorXd Y){
			// Predicts the output Y with the predicted X
			Y_pred = H * X;

			//Calculates prediction error (also called measurement innovation or residual)
			I_pred = Y - Y_pred;

			// Kalman gain
			K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

			// Updates estimated state matrix with Kalman gain and estimated output error
			X = X + K * I_pred;

			ROS_INFO("K:");
			for (int i = 0; i < K.rows(); ++i) {
				std::string row_str = "[ ";
				for (int j = 0; j < K.cols(); ++j) {
					row_str += std::to_string(K(i, j)) + " ";
				}
				row_str += "]";
				ROS_INFO_STREAM(row_str);
			}

			for(int i=0;i < I_pred.size();i++){
				ROS_INFO("i_pred[%d] = %f",i, I_pred[i]);
			}

			for(int i=0;i < X.size();i++){
				ROS_INFO("new x%d = %f",i, X[i]);
			}

			// Updating covariance error matrix with Kalman gaind and old covariance error matrix
			P = (Eigen::MatrixXd::Identity(X.rows(), X.rows()) - K * H) * P;

			// New prediction of output
			Y_cor = H * X;

			// New error of output
			I_cor = Y - Y_cor;
		}

		Eigen::VectorXd getState() { return X; }
		Eigen::MatrixXd getCovariance() { return P; }
		Eigen::VectorXd getPredictedOutput() { return Y_pred; }
		Eigen::VectorXd getCorrectedOutput() { return Y_cor; }
		Eigen::VectorXd getPredictedInnovation() { return I_pred; }
		Eigen::VectorXd getCorrectedInnovation() { return I_cor; }
};

#endif