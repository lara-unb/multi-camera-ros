#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

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

#define ARBITRARY_INITIAL_COVARIANCE 1

class filter{

	public:
		Eigen::MatrixXd A;  // State transition matrix, presumably constant and equal identity
		Eigen::MatrixXd H;  // Observation/measurement matrix, presumably constant and equal identity
		Eigen::MatrixXd Q;  // Process noise covariance
		Eigen::MatrixXd R;  // Measurement noise covariance
		Eigen::MatrixXd X0; // Initial state vector
		Eigen::MatrixXd P0; // Initial covariance matrix
		Eigen::MatrixXd X; 	// State vector
		Eigen::MatrixXd P; 	// Covariance matrix
		Eigen::MatrixXd K;	// Kalman filter gain
		Eigen::MatrixXd Y_pred; // Predicted output
		Eigen::MatrixXd Y_cor;	// Corrected output
		Eigen::MatrixXd I_cor;	// Corrected measurement innovation/residual
		Eigen::MatrixXd I_pred; // Predicterd measurement innovation/residual

		void setDimensions(int Nx, int Ny){
			A.setIdentity(Nx, Nx);
			H.resize(Ny, Nx);
			Q.resize(Nx, Nx);
			R.resize(Ny, Ny);
			X0.resize(Nx, 1);
			P0.resize(Nx, Nx);
			X.resize(Nx, 1);
			P.resize(Nx, Nx);
			I_pred.resize(Ny, 1);
			I_cor.resize(Ny, 1);
			K.resize(Nx, Ny);
		}

		void initNormalDist(int kQ, int kR){
			Q = Eigen::MatrixXd::Identity(Q.rows(), Q.rows()) * kQ;
			R = Eigen::MatrixXd::Identity(R.rows(), R.rows()) * kR;
		}

		void reset(){
			X = X0;
			P = P0 * ARBITRARY_INITIAL_COVARIANCE;
		}

		void predict(){
			// Predicst state vector
			X = A * X;

			// Predicts Covariance error matrix
			P = A * P * A.transpose() + Q;
		}

		void correct(Eigen::MatrixXd& Y){
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

		Eigen::MatrixXd getState() { return X; }
		Eigen::MatrixXd getCovariance() { return P; }
		Eigen::MatrixXd getPredictedOutput() { return Y_pred; }
		Eigen::MatrixXd getCorrectedOutput() { return Y_cor; }
		Eigen::MatrixXd getPredictedInnovation() { return I_pred; }
		Eigen::MatrixXd getCorrectedInnovation() { return I_cor; }
};

#endif