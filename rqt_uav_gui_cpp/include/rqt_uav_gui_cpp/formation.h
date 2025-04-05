#ifndef FORMATION_H
#define FORMATION_H
#include <math.h>
#include <algorithm>
#include <vector>

class formation {
	private:
		std::vector<std::vector<double>> points;
		std::vector<std::vector<double>> cur_points;
		size_t number=0;
		double parameter=0, sector_angle=0, arc_length=0;
		double x_first=0, y_first=0, z_first=0;
		double angle_rotation_form=0;

	public:
	
		formation();
		formation(const size_t number, const double parameter, const double sector_angle, const double arc_length, const double x_first, const double y_first, const double z_first);
		~formation()=default;
		void setXYZFirst(const double x_first, const double y_first, const double z_first);
		double getZFirst();
		size_t getNumber();
		double getParameter();
		void setNumber(const size_t number);
		void setParameter(const double parameter);
		void changeRotationAngle(const double new_angle);
		double getRotationAngle();
		void setSectorAngle(const double sector_angle);
		void setArcLength(const double arc_length);
		void setFormRotAngle(const double rotangle);
		void applyRotation();
		std::vector<std::vector<double>> getPoints();
		bool check_for_collisions(const std::vector<double>& pos, const std::vector<std::vector<double>>& allPos);
		void generate_points_circle();
		void generate_points_snake();
		void generate_points_front();
		void generate_points_column();
		void generate_points_peleng();
		void generate_points_flangs();
		void generate_points_chess();
		void generate_points_rect_full();
		void generate_points_rect_empty();
		void generate_points_klin();
		void generate_points_klin_with_first();
		void generate_points_romb();
		void generate_points_romb_empty();
		void generate_points_sector();
};
#endif
