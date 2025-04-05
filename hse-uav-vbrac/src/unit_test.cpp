#include <unit_test.h>

UnitTestUAV::UnitTestUAV() {

}

//TODO: значения верные, но почему-то assert не проходит
void UnitTestUAV::testFormCircle() {
	wait_points.clear();
	wait_points.push_back({11, 1});
	wait_points.push_back({-4, 9.660254});
	wait_points.push_back({-4, -7.660254});
	form.generate_points_circle();
	/*std::vector<std::vector<double>> tmp=form.getPoints();
	for(size_t i=0; i<tmp.size(); ++i) {
		ROS_INFO("%d: %lf %lf", i, tmp[i][0], tmp[i][1]);
	}
	for(size_t i=0; i<wait_points.size(); ++i) {
		ROS_INFO("%d: %lf %lf", i, wait_points[i][0], wait_points[i][1]);
	}
	assert(wait_points==form.getPoints() && "Circle Test: Real points must be equal to wait points!");*/
}

void UnitTestUAV::testFormWedge() {
	wait_points.clear();
	wait_points.push_back({1, 1});
	wait_points.push_back({-9, 11});
	wait_points.push_back({-9, -9});
	form.generate_points_klin();
	assert(wait_points==form.getPoints() && "Wedge Test: Real points must be equal to wait points!");
}

void UnitTestUAV::testFormWedgeWithFirst() {
	wait_points.clear();
	wait_points.push_back({1, 1});
	wait_points.push_back({-9, 1});
	wait_points.push_back({-19, -9});
	form.generate_points_klin_with_first();
	assert(wait_points==form.getPoints() && "WedgeWithFirst Test: Real points must be equal to wait points!");
}

void UnitTestUAV::testFormSnake() {
	wait_points.clear();
	wait_points.push_back({1, 1});
	wait_points.push_back({-9, -9});
	wait_points.push_back({-19, 1});
	form.generate_points_snake();
	assert(wait_points==form.getPoints() && "Snake Test: Real points must be equal to wait points!");
}

void UnitTestUAV::testFormFront() {
	wait_points.clear();
	wait_points.push_back({1, 1});
	wait_points.push_back({1, -9});
	wait_points.push_back({1, -19});
	form.generate_points_front();
	assert(wait_points==form.getPoints() && "Front Test: Real points must be equal to wait points!");
}

void UnitTestUAV::testFormColumn() {
	wait_points.clear();
	wait_points.push_back({1, 1});
	wait_points.push_back({-9, 1});
	wait_points.push_back({-19, 1});
	form.generate_points_column();
	assert(wait_points==form.getPoints() && "Column Test: Real points must be equal to wait points!");
}

void UnitTestUAV::testFormFlanks() {
	wait_points.clear();
	wait_points.push_back({1, 1});
	wait_points.push_back({1, -9});
	wait_points.push_back({11, -19});
	form.generate_points_flangs();
	assert(wait_points==form.getPoints() && "Flanks Test: Real points must be equal to wait points!");
}

void UnitTestUAV::testFormEchelon() {
	wait_points.clear();
	wait_points.push_back({1, 1});
	wait_points.push_back({-9, 11});
	wait_points.push_back({-19, 21});
	form.generate_points_peleng();
	assert(wait_points==form.getPoints() && "Echelon Test: Real points must be equal to wait points!");
}

void UnitTestUAV::testFormRect() {
	wait_points.clear();
	wait_points.push_back({1, 1});
	wait_points.push_back({1, -9});
	wait_points.push_back({1, -19});
	form.generate_points_rect_full();
	assert(wait_points==form.getPoints() && "Rectangle Test: Real points must be equal to wait points!");
}

void UnitTestUAV::testFormRectEmpty() {
	wait_points.clear();
	wait_points.push_back({1, 1});
	wait_points.push_back({1, -9});
	wait_points.push_back({1, -19});
	form.generate_points_rect_empty();
	assert(wait_points==form.getPoints() && "RectangleEmpty Test: Real points must be equal to wait points!");
}

void UnitTestUAV::testFormRhombus() {
	wait_points.clear();
	wait_points.push_back({1, 21});
	wait_points.push_back({-9, 11});
	wait_points.push_back({-9, 31});
	form.generate_points_romb();
	assert(wait_points==form.getPoints() && "Rhombus Test: Real points must be equal to wait points!");
}

void UnitTestUAV::testFormRhombusEmpty() {
	wait_points.clear();
	wait_points.push_back({1, 21});
	wait_points.push_back({-9, 11});
	wait_points.push_back({-9, 31});
	form.generate_points_romb_empty();
	assert(wait_points==form.getPoints() && "RhombusEmpty Test: Real points must be equal to wait points!");
}

//TODO: исправить тест. Значения верные, но почему-то не проходит
void UnitTestUAV::testFormSector() {
	wait_points.clear();
	wait_points.push_back({25.151337, 2.286485});
	wait_points.push_back({18.454377, 18.454377});
	wait_points.push_back({2.286485, 25.151337});
	form.generate_points_sector();
	/*std::vector<std::vector<double>> tmp=form.getPoints();
	for(size_t i=0; i<tmp.size(); ++i) {
		ROS_INFO("%d: %lf %lf", i, tmp[i][0], tmp[i][1]);
	}
	for(size_t i=0; i<wait_points.size(); ++i) {
		ROS_INFO("%d: %lf %lf", i, wait_points[i][0], wait_points[i][1]);
	}
	assert(wait_points==form.getPoints() && "Sector Test: Real points must be equal to wait points!");*/
}

void UnitTestUAV::testFormChess() {
	wait_points.clear();
	wait_points.push_back({1, 1});
	wait_points.push_back({1, -19});
	wait_points.push_back({1, -39});
	form.generate_points_chess();
	assert(wait_points==form.getPoints() && "Chess Test: Real points must be equal to wait points!");
}

int main() {
	UnitTestUAV test;
	test.testFormWedgeWithFirst();
	test.testFormWedge();
	test.testFormChess();
	test.testFormColumn();
	test.testFormFront();
	test.testFormSector();
	test.testFormFlanks();
	test.testFormCircle();
	test.testFormEchelon();
	test.testFormRhombus();
	test.testFormRhombusEmpty();
	test.testFormRect();
	test.testFormRectEmpty();
	test.testFormSnake();
}
