#pragma once

struct grid {
public:
	grid():m_id(0), m_x(0), m_y(0), m_g(0), m_h(0),m_f(0) {}
	grid(int id, int x, int y) :m_id(id), m_x(x), m_y(y), m_g(0), m_h(0), m_f(0) {}
	bool operator>(const grid& g) {
		return m_f > g.m_f;
	}
	void clean_weight() {
		m_f = 0;
		m_g = 0;
		m_h = 0;
	}
public:
	int m_id;
	//���Ǹ����ڶ�ά���������
	int m_x;
	int m_y;
	// F = G+H
	int m_g; //�Ե�ǰ��Ϊ����,���ܱ߽�������; ֱ�߱�Ϊ10,б�߱�Ϊ14(��ֱ1,б sqrt(1^2+1^2),��2����,Ϊ�������,����������)
	int m_h; //���ӵ��յ���ӵ�ֱ�߾���(��������ĸ�������*10)
	int m_f;
};

bool operator==(const grid& g1, const grid& g2) {
	return g1.m_x == g2.m_x && g1.m_y == g2.m_y;
}

bool operator!=(const grid& g1, const grid& g2) {
	return !(g1 == g2);
}

struct cmp {
public:
	bool operator()(grid g1, grid g2) {
		return g1 > g2;
	}
};
