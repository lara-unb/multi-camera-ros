#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#define POSE_VECTOR_SIZE 7
#define LINEAR_VECTOR_SIZE 3
#define ANGULAR_VECTOR_SIZE 4
#define HIGH_COVARIANCE_PRESET 0.1
#define PROCESS_LINEAR_NOISE_COVARIANCE 0.01
#define PROCESS_ANGULAR_NOISE_COVARIANCE 0.000001

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

		void set_dimensions(int dim){
			X.conservativeResize(dim);
			X0.conservativeResize(dim);
			P.conservativeResize(dim, dim);
			P0.conservativeResize(dim, dim);
			A.setIdentity(dim, dim);
			H.setIdentity(dim, dim);
			Q.conservativeResize(dim, dim);
			Q0.conservativeResize(POSE_VECTOR_SIZE, POSE_VECTOR_SIZE);
			Q0.topLeftCorner(LINEAR_VECTOR_SIZE, LINEAR_VECTOR_SIZE) = Eigen::MatrixXd::Identity(LINEAR_VECTOR_SIZE, LINEAR_VECTOR_SIZE) * PROCESS_LINEAR_NOISE_COVARIANCE;
			Q0.bottomRightCorner(ANGULAR_VECTOR_SIZE, ANGULAR_VECTOR_SIZE) = Eigen::MatrixXd::Identity(ANGULAR_VECTOR_SIZE, ANGULAR_VECTOR_SIZE) * PROCESS_ANGULAR_NOISE_COVARIANCE;
			R.conservativeResize(dim, dim);
			R0 = Eigen::MatrixXd::Identity(POSE_VECTOR_SIZE, POSE_VECTOR_SIZE);
		}

		void insertState(Eigen::VectorXd state){

			// Regarding the state vector
			X.conservativeResize(X.rows() + POSE_VECTOR_SIZE);
			X.tail(state.size()) = state;

			// Regarding the Covariance matrix
			P.conservativeResize(P.rows() + POSE_VECTOR_SIZE, P.cols() + POSE_VECTOR_SIZE);
			P.bottomRightCorner(POSE_VECTOR_SIZE, POSE_VECTOR_SIZE) = Eigen::MatrixXd::Identity(POSE_VECTOR_SIZE, POSE_VECTOR_SIZE) * HIGH_COVARIANCE_PRESET;

			// Regarding the observation matrix
			A = Eigen::MatrixXd::Identity(A.rows() + POSE_VECTOR_SIZE, A.cols() + POSE_VECTOR_SIZE);

			// Regarding the observation matrix
			H = Eigen::MatrixXd::Identity(H.rows() + POSE_VECTOR_SIZE, H.cols() + POSE_VECTOR_SIZE);

			// Regarding the process noise covariance matrix
			Q.conservativeResize(Q.rows() + POSE_VECTOR_SIZE, Q.cols() + POSE_VECTOR_SIZE);
			Q.bottomRightCorner(POSE_VECTOR_SIZE, POSE_VECTOR_SIZE) = Q0;

			// Regarding the measurement noise covariance matrix
			R = Eigen::MatrixXd::Identity(R.rows() + POSE_VECTOR_SIZE, R.cols() + POSE_VECTOR_SIZE);
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

			// Predicts Covariance error matrix
			P = A * P * A.transpose() + Q;
		}

		void correct(Eigen::VectorXd& Y){
			// Predicts the output Y with the predicted X
			Y_pred = H * X;

			//Calculates prediction error (also called measurement innovation or residual)
			I_pred = Y - Y_pred;

			// Kalman gain
			K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

			// Updates estimated state matrix with Kalman gain and estimated output error
			X = X + K * I_pred;

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