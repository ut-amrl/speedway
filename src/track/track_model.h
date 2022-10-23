#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace track {

class TrackModel {
 public:
  TrackModel();
  void UpdatePointcloud(const std::vector<Eigen::Vector2f>& cloud);

 protected:
  void SegmentPointcloud();
  void FitWallPolynomials();

  std::vector<Eigen::Vector2f> cloud_;
  std::vector<Eigen::Vector2f> left_wall_points_;
  std::vector<Eigen::Vector2f> right_wall_points_;

  std::vector<double> left_wall_t;
  std::vector<double> left_x_coeffs;
  std::vector<double> left_y_coeffs;
  std::vector<double> right_wall_t;
  std::vector<double> right_x_coeffs;
  std::vector<double> right_y_coeffs;
};

void PolynomialRegression(const std::vector<double>& t,
                          const std::vector<double>& v,
                          std::vector<double>& coeffs, const uint32_t order);

double EvaluatePolynomial(const double t, const std::vector<double>& coeffs);

}  // namespace track
