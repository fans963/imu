#include "eigen3/Eigen/Eigen"

namespace filter {
    class KalmanFilter {
        public:
            KalmanFilter();
            ~KalmanFilter();
            void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);
            void predict();
            void update(const Eigen::VectorXd& z);
            Eigen::VectorXd getState();
            Eigen::MatrixXd getCovariance();
        private:
            Eigen::VectorXd x;
            Eigen::MatrixXd P;
            Eigen::MatrixXd F;
            Eigen::MatrixXd Q;
            Eigen::MatrixXd H;
            Eigen::MatrixXd R;
    };
}