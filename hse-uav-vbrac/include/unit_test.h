#ifndef TESTUAV_H
#define TESTUAV_H
#undef NDEBUG
#include <uav.h>
#include <formation.h>
#include <assert.h>
#include <vector>
#include <algorithm>


class UnitTestUAV {
	private:
		//for formation tests part
		formation form{3, 10, 90, 35, 1, 1, 1};
		std::vector<std::vector<double>> wait_points;
		//
		
	public:
		//Tests for formation
		UnitTestUAV();
		~UnitTestUAV()=default;
		void testFormCircle();
		void testFormWedge();
		void testFormWedgeWithFirst();
		void testFormSnake();
		void testFormFront();
		void testFormColumn();
		void testFormFlanks();
		void testFormEchelon();
		void testFormRect();
		void testFormRectEmpty();
		void testFormRhombus();
		void testFormRhombusEmpty();
		void testFormSector();
		void testFormChess();
		
		//Other tests

};
#endif

