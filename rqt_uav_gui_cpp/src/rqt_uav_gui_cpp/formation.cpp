#include <rqt_uav_gui_cpp/formation.h>

formation::formation() {
	number=0;
	parameter=0;
	sector_angle=0;
	arc_length=0;
	x_first=1;
	y_first=1;
	z_first=0;
}

formation::formation(const size_t number, const double parameter, const double sector_angle, const double arc_length, const double x_first, const double y_first, const double z_first) {
	this->number=number;
	this->parameter=parameter;
	this->sector_angle=sector_angle;
	this->arc_length=arc_length;
	this->x_first=x_first;
	this->y_first=y_first;
	this->z_first=z_first;
	angle_rotation_form=0;
}

std::vector<std::vector<double>> formation::getPoints() {
	//return points;
	return cur_points;
}

double formation::getZFirst() {
	return z_first;
}

void formation::setXYZFirst(const double x_first, const double y_first, const double z_first) {
	this->x_first=x_first;
	this->y_first=y_first;
	this->z_first=z_first;
}

void formation::setNumber(const size_t number) {
	this->number=number;
}

void formation::setParameter(const double parameter) {
	this->parameter=parameter;
}

void formation::setSectorAngle(const double sector_angle) {
	this->sector_angle=sector_angle;
}

void formation::setArcLength(const double arc_length) {
	this->arc_length=arc_length;
}

//проверка на наличие коллизий - занята ли данная позиция другим дроном, актуально для окружностей! false - точка свободна, true - точка занята
bool formation::check_for_collisions(const std::vector<double>& pos, const std::vector<std::vector<double>>& allPos) {
	auto result{std::find(begin(allPos), end(allPos), pos)};
	if(result==end(allPos))
		return false;
	else
		return true;
}

//Generate circle formation
void formation::generate_points_circle() {
	points.clear();
	double angle=0.0;
	double x=0.0;
	double y=0.0;
	std::vector<double> point;
	for(size_t i=0; i<number; ++i) {
		angle=2*M_PI*i/number;
		x=x_first+parameter*cos(angle);
		y=y_first+parameter*sin(angle);
		point={x, y};
//		while(1) {
			if(!check_for_collisions(point, points)) {
				points.push_back({x, y});
//				break;
			}
//		}
	}
	//return points;
	//cur_points=points;
	applyRotation();
}

//Generate snake formation
void formation::generate_points_snake() {
	points.clear();
	double x=x_first;
	double y=y_first;
	points.push_back({x, y});
	for(size_t i=1; i<number; ++i) {
		if(i%2==0) {
			y+=parameter;
		} else {
			y-=parameter;
		}
		x-=parameter;
		points.push_back({x, y});
	}
	//return points;
	//cur_points=points;
	applyRotation();
}

//Generate front formation
void formation::generate_points_front() {
	points.clear();
	double x=x_first;
	double y=y_first;
	points.push_back({x, y});
	for(size_t i=1; i<number; ++i) {
		y-=parameter;
		points.push_back({x, y});
	}
	//return points;
	//cur_points=points;
	applyRotation();
}

//Generate column formation
void formation::generate_points_column() {
	points.clear();
	double x=x_first;
	double y=y_first;
	points.push_back({x,y});
	for(size_t i=1; i<number; ++i) {
		x-=parameter;
		points.push_back({x, y});
	}
	//return points;
	//cur_points=points;
	applyRotation();
}

//Generate peleng formation
void formation::generate_points_peleng() {
	points.clear();
	double x=x_first;
	double y=y_first;
	points.push_back({x,y});
	for(size_t i=1; i<number; ++i) {
		x-=parameter;
		y+=parameter;
		points.push_back({x, y});
	}
	//return points;
	//cur_points=points;
	applyRotation();
}

//Generate flangs formation
void formation::generate_points_flangs() {
	points.clear();
	double x=x_first;
	double y=y_first;
	points.push_back({x, y});
	for(size_t i=1; i<number/2; ++i) { //first line - left
		x+=parameter;
		y+=parameter;
		points.push_back({x, y});
	}
	x=x_first;
	y=y_first-parameter;
	points.push_back({x, y});
	for(size_t i=(number/2)+1; i<number; ++i) {
		x+=parameter;
		y-=parameter;
		points.push_back({x, y});
	}
	//return points;
	//cur_points=points;
	applyRotation();
}

//Generate chess formation
void formation::generate_points_chess() {
	points.clear();
	double x=x_first;
	double y=y_first;
	int amount=number;
	bool flag=false;
	while(amount>0) {
		for(size_t j=0; j<6; ++j) { //6 columns
			if(amount==0)
				break;
			if(flag==false&&j%2==0) {
				points.push_back({x,y});
				amount-=1;
			} else if(flag==true&&j%2!=0) {
				points.push_back({x, y});
				amount-=1;
			}
			y-=parameter;
		}
		if(flag==true)
			flag=false;
		else
			flag=true;
		y=y_first;
		x-=parameter;
	}
	//return points;
	//cur_points=points;
	applyRotation();
}

//Generate rectangle full formation: 6 columns, lines while amount>0
void formation::generate_points_rect_full() {
	points.clear();
	double x=x_first;
	double y=y_first;
	int amount=number;
	while(amount>0) {
		for(size_t j=0; j<6; ++j) { //6 columns
			if(amount==0)
				break;
			points.push_back({x, y});
			y-=parameter;
			amount-=1;
		}
		y=y_first;
		x-=parameter;
	}
	//cur_points=points;
	applyRotation();
}

//Generate rectangle empty formation: 6 columns only firt and last line( last - if we have amount<=5, because one UAV will be always if we have it)
void formation::generate_points_rect_empty() {
	points.clear();
	double x=x_first;
	double y=y_first;
	int amount=number;
	bool isFirstLine=true;
	while(amount>0) {
		for(size_t j=0; j<6; ++j) { //6 columns
			if (amount==0) {
				break;
			} else if(j==0||j==5||isFirstLine) { //5 - last column, 0 - first column in line
				points.push_back({x, y});
				amount-=1;
			} else if(amount-5<=0) {
				points.push_back({x, y});
				amount-=1;
			}
			y-=parameter;
		}
		y=y_first;
		isFirstLine=false;
		x-=parameter;
	}
	/*for(size_t i=0; i<lines; ++i) {
		for(size_t j=0; j<columns; ++j) {
			if(i==0||i==lines-1) {
				points.push_back({x, y});
			} else if(j==0||j==columns-1) {
				points.push_back({x, y});
			}
			y-=parameter;
		}
		y=y_first;
		x-=parameter;
	}*/
	//cur_points=points;
	applyRotation();
}

//Generate klin formation
void formation::generate_points_klin() {
	points.clear();
	double x=x_first;
	double y=y_first;
	size_t flag=0;
	points.push_back({x, y});
	for(size_t i=0; i<number-1; ++i) {
		if(i<(number-1)/2) { //left side
			x-=parameter;
			y+=parameter;
		} else { //right side
			if(flag==0) {
				x=x_first;
				y=y_first;
				flag=1;
			}
			x-=parameter;
			y-=parameter;
		}
		points.push_back({x,y});
	}
	//return points;
	/*std::vector<std::vector<double>> points_new=points;
	for(size_t i=0; i<points_new.size(); ++i) {
		points_new[i][0]=points[i][0]*cos(angle_rotation_form)-points[i][1]*sin(angle_rotation_form);
		points_new[i][1]=points[i][0]*sin(angle_rotation_form)+points[i][1]*cos(angle_rotation_form);
	}
	cur_points=points_new;*/
	applyRotation();
	//cur_points=points;
}

//Generate klin with first formation
void formation::generate_points_klin_with_first() {
	points.clear();
	double x=x_first;
	double y=y_first;
	size_t flag=0;
	points.push_back({x, y});
	x-=parameter;
	points.push_back({x, y});
	for(size_t i=0; i<number-2; ++i) {
		if(i<(number-2)/2) { //left side
			x-=parameter;
			y+=parameter;
		} else { //right side
			if(flag==0) {
				x=x_first-parameter;
				y=y_first;
				flag=1;
			}
			x-=parameter;
			y-=parameter;
		}
		points.push_back({x,y});
	}
	//return points;
	//points_cur=points;
	applyRotation();
}

//Generate romb full formation
void formation::generate_points_romb() {
	points.clear();
	double x=x_first;
	double y=y_first;
	int i, j;
	int m=number;
	int n=ceil(sqrt(m));
	int c=0, inc=1;
	for(i=1; i<=(n*2)-1; ++i) {
		c+=inc;
		if(i==n)
			inc=-1;
		y+=parameter*(n-c);
		for(j=1; j<=c; ++j) {
			if(m>0) {
				y+=parameter;
				points.push_back({x,y});
				y+=parameter;
				m-=1;
			}
		}
		x-=parameter;
		y=y_first;
	}
	//cur_points=points;
	applyRotation();
}

//Generate romb empty formation
void formation::generate_points_romb_empty() {
	points.clear();
	double x=x_first;
	double y=y_first;
	int i, j;
	int m=number;
	int n=ceil(sqrt(m));
	int c=0, inc=1;
	
	if(n>2)
		n+=1;
	for(i=1; i<=(n*2)-1; ++i) {
		c+=inc;
		if(i==n)
			inc=-1;
		y+=parameter*(n-c);
		for(j=1; j<=c; ++j) {
			y+=parameter;
			if(m>0&&(j==1||j==c)) {
				points.push_back({x,y});
				m-=1;
			}
			y+=parameter;
		}
		x-=parameter;
		y=y_first;
	}
	//cur_points=points;
	applyRotation();
}

//Generate circle sector formation
//parameter - radius, parameter2 - arc_length - более не используется и высчитывается автоматически. parameter3 - sector_angle
void formation::generate_points_sector() {
	std::vector<double> theta;
	//std::vector<double> distances;
	points.clear();
	std::vector<std::vector<double>> filtered_points;
	double dx=0.0;
	double dy=0.0;
	double distance=0.0;
	double scale_factor=0.0;
	double dist_sum=0.0;
	for(size_t i=0; i<number; ++i) {
		theta.push_back(i*(sector_angle*M_PI/180)/(number-1));
	}
	
	for(size_t i=0; i<theta.size(); ++i) {
		points.push_back({x_first+parameter*cos(theta[i]), y_first+parameter*sin(theta[i])});
	}
	
	for(size_t i=1; i<number; ++i) {
		dx=points[i][0]-points[i-1][0];
		dy=points[i][1]-points[i-1][1];
		distance=sqrt(pow(dx, 2)+pow(dy, 2));
		//distances.push_back(distance);
		dist_sum+=distance;
	}
	
	scale_factor=(sector_angle/360)*2*M_PI*parameter/dist_sum;//arc_length/dist_sum;
	for(size_t i=0; i<points.size(); ++i) {
		filtered_points.push_back({points[i][0]*scale_factor, points[i][1]*scale_factor});
	}
	points.clear();
	std::copy(filtered_points.begin(), filtered_points.end(), std::back_inserter(points));
	//return filtered_points;
	//cur_points=points;
	applyRotation();
}

void formation::applyRotation() {
	std::vector<std::vector<double>> points_new=points;
	for(size_t i=0; i<points_new.size(); ++i) {
		points_new[i][0]=points[i][0]*cos(angle_rotation_form)-points[i][1]*sin(angle_rotation_form);
		points_new[i][1]=points[i][0]*sin(angle_rotation_form)+points[i][1]*cos(angle_rotation_form);
	}
	cur_points=points_new;
}

void formation::changeRotationAngle(const double new_angle) {
	angle_rotation_form=new_angle;//*M_PI/180;
	std::vector<std::vector<double>> points_new=points;
	for(size_t i=0; i<points_new.size(); ++i) {
		points_new[i][0]=points[i][0]*cos(angle_rotation_form)-points[i][1]*sin(angle_rotation_form);
		points_new[i][1]=points[i][0]*sin(angle_rotation_form)+points[i][1]*cos(angle_rotation_form);
	}
	cur_points=points_new;
}

void formation::setFormRotAngle(const double rotangle) {
	angle_rotation_form+=rotangle*M_PI/180;
}

double formation::getRotationAngle() {
	return angle_rotation_form;
}

size_t formation::getNumber() {
	return number;
}

double formation::getParameter() {
	return parameter;
}
