#include <iostream>
#include <string.h>
#include <math.h>

using namespace std;

class Matrix
{
private:
	int row, col;
	float* index;
public:
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>构 造 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// 指定行列数构造
	Matrix(int X = 1, int Y = 1) {
		row = X;
		col = Y;
		index = (float *)malloc( row*col*sizeof( float ));
	}
	// Matrix类构造
	Matrix(const Matrix& obj) {
		row = obj.row;
		col = obj.col;
		index = (float *)malloc( row*col*sizeof( float ));
		memcpy(index, obj.index, row*col* sizeof( float ));
	}
	// 二维数组构造
	Matrix(float **array, int X, int Y) {
		row = X;
		col = Y;
		index = (float *)malloc( row*col*sizeof( float ));
		for (int i = 0; i < row; ++i){
			memcpy(index + i*col, array[i], col*sizeof( float ));
		}
	}
	// 连续数组构造
	Matrix(float *array, int X, int Y) {
		row = X;
		col = Y;
		index = (float *)malloc( row*col*sizeof( float ));
		memcpy(index, array, row*col* sizeof( float ));
	}
	// 复制构造
	Matrix& operator=(const Matrix& obj) {
		free(index);
		row = obj.row;
		col = obj.col;
		index = (float *)malloc( row*col*sizeof( float ));
		memcpy(index, obj.index, row*col*sizeof( float ));
		return *this;
	}
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>析 构 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	~Matrix() {
		free(index);
	}
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>初 始 化<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// 单位阵初始化
	void E() {
		memset(index, 0, row*col*sizeof(float));
		for (int i = 0; i < row; ++i) {
			index[i + i*col] = 1;
		}
	}
	// 全零初始化
	void zero() {
		memset(index, 0, row*col*sizeof(float));
	}
	// 输入赋值
	friend istream& operator>>(istream& is, Matrix& obj) {
		for (int i = 0; i < obj.row*obj.col; ++i) {
			is >> obj.index[i];
		}
		return is;
	}
	// 输出
	friend ostream& operator<<(ostream& os, const Matrix& obj) {
		for (int i = 0; i < obj.row; ++i) {
			for (int j = 0; j < obj.col; ++j) {
				os << obj.index[i*obj.col + j] << ' ';
			}
			os << endl;
		}
		return os;
	}
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>成 员 获 取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// 行数
	int Row() {
		return row;
	}
	// 列数
	int Col() {
		return col;
	}
	float *Index(){
		return index;
	}
	// 索引元素
	float& operator()(int X, int Y) {
		return index[X*col + Y];
	}
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>基 本 运 算<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	friend Matrix operator+(const Matrix& obj1, const Matrix& obj2) {
		if (obj1.row != obj2.row || obj1.col != obj2.col) {
			printf("相加矩阵维数不同！\n");
			return obj1;
		}
		else {
			Matrix tmp(obj1.row, obj1.col);
			for (int i = 0; i < tmp.row*tmp.col; ++i) {
				tmp.index[i] = obj1.index[i] + obj2.index[i];
			}
			return tmp;
		}
	}
	friend Matrix operator+(const Matrix& obj, const float X) {
		Matrix tmp(obj.row, obj.col);
		for (int i = 0; i < tmp.row*tmp.col; ++i) {
			tmp.index[i] = obj.index[i] + X;
		}
		return tmp;
	}
	friend Matrix operator+(const float X, const Matrix& obj) {
		Matrix tmp(obj.row, obj.col);
		for (int i = 0; i < tmp.row*tmp.col; ++i) {
			tmp.index[i] = obj.index[i] + X;
		}
		return tmp;
	}
	friend Matrix operator-(const Matrix& obj1, const Matrix& obj2) {
		if (obj1.row != obj2.row || obj1.col != obj2.col) {
			printf("相减矩阵维数不同！\n");
			return obj1;
		}
		else {
			Matrix tmp(obj1.row, obj1.col);
			for (int i = 0; i < tmp.row*tmp.col; ++i) {
				tmp.index[i] = obj1.index[i] - obj2.index[i];
			}
			return tmp;
		}
	}
	friend Matrix operator-(const Matrix& obj, const float X) {
		Matrix tmp(obj.row, obj.col);
		for (int i = 0; i < tmp.row*tmp.col; ++i) {
			tmp.index[i] = obj.index[i] - X;
		}
		return tmp;
	}
	friend Matrix operator-(const float X, const Matrix& obj) {
		Matrix tmp(obj.row, obj.col);
		for (int i = 0; i < tmp.row*tmp.col; ++i) {
			tmp.index[i] = X - obj.index[i];
		}
		return tmp;
	}
	friend Matrix operator*(const Matrix& obj1, const Matrix& obj2) {
		if (obj1.col != obj2.row) {
			printf("维数不符合矩阵相乘原则！\n");
			return obj1;
		}
		else {
			Matrix tmp(obj1.row, obj2.col);
			for (int i = 0; i < tmp.row; ++i) {
				for (int j = 0; j < tmp.col; ++j) {
					tmp.index[i*tmp.col + j] = 0;
					for (int k = 0; k < obj1.col; ++k) {
						tmp.index[i*tmp.col + j] += obj1.index[i*obj1.col + k] * obj2.index[k*obj2.col + j];
					}
				}
			}
			return tmp;
		}
	}
	friend Matrix operator*(const Matrix& obj, const float X) {
		Matrix tmp(obj.row, obj.col);
		for (int i = 0; i < tmp.row*tmp.col; ++i) {
			tmp.index[i] = obj.index[i] * X;
		}
		return tmp;
	}
	friend Matrix operator*(const float X, const Matrix& obj) {
		Matrix tmp(obj.row, obj.col);
		for (int i = 0; i < tmp.row*tmp.col; ++i) {
			tmp.index[i] = obj.index[i] * X;
		}
		return tmp;
	}
	friend Matrix operator/(const Matrix& obj, const float X) {
		Matrix tmp(obj.row, obj.col);
		for (int i = 0; i < tmp.row*tmp.col; ++i) {
			tmp.index[i] = obj.index[i] / X;
		}
		return tmp;
	}
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>矩 阵 变 换<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	//矩阵转置
	Matrix trans() {
		Matrix tmp(col, row);
		for (int i = 0; i < tmp.row; ++i) {
			for (int j = 0; j < tmp.col; ++j) {
				tmp.index[i*tmp.col + j] = index[j*col + i];
			}
		}
		return tmp;
	}
	//分块矩阵
	Matrix block(int X, int Y, int new_row, int new_col) {
		Matrix tmp(new_row, new_col);
		for (int i = 0; i < new_row; ++i) {
			memcpy(tmp.index + i*new_col, index + (i + X)*col + Y, new_col* sizeof( float ));
		}
		return tmp;
	}
	// 矩阵赋值
	void cpy(Matrix obj, int X, int Y) {
		for (int i = 0; i < obj.row; ++i) {
			memcpy(index + (i + X)*col + Y, obj.index + i*obj.col, obj.col* sizeof( float ));
		}
	}
	//行交换
	void swaprow(const int X, const int Y) {
		float* tmp = (float *)malloc( col*sizeof( float ));
		memcpy(tmp, index + X*col, col* sizeof( float ));
		memcpy(index + X*col, index + Y*col, col* sizeof( float ));
		memcpy(index + Y*col, tmp, col* sizeof( float ));
	}
	//行变换
	void rowmultik(int X, float Y) {
		for (int i = 0; i < col; ++i) {
			index[X*col + i] *= Y;
		}
	}
	void exp(int X){
		for (int i = 0; i < row*col; ++i){
			index[i] = pow(index[i], X);
		}
	}
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>复 杂 运 算<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	//求矩阵任意元素的余子式
	float cofactor(const int X, const int Y) {
		float tmp;
		Matrix M1(row - 1, row - 1);
		for (int i = 0; i < row; ++i) {
			if (i == X) {
				continue;
			}
			for (int j = 0; j < col; ++j) {
				if (j == Y) {
					continue;
				}
				if (i < X) {
					if (j < Y) {
						M1.index[i*M1.col + j] = index[i*col + j];
					}
					else {
						M1.index[i*M1.col + j - 1] = index[i*col + j];
					}
				}
				else {
					if (j < Y) {
						M1.index[(i - 1)*M1.col + j] = index[i*col + j];
					}
					else {
						M1.index[(i - 1)*M1.col + j - 1] = index[i*col + j];
					}
				}
			}
		}
		tmp = M1.value();
		return tmp;
	}
	//求行列式的值
	float value() {
		Matrix tmp(*this);
		int count = 0;
		int i, j, k;
		float det = 1, tp;
		for (i = 0; i < row; ++i) {
			if (tmp.index[i*tmp.col + i] == 0) {
				for (j = i + 1; j < col; ++j) {
					if (tmp.index[j*tmp.col + i] != 0) {
						tmp.swaprow(i, j);
						++count;
						break;
					}
				}
				if (j == col) {
					return 0;
				}
			}
			for (j = i + 1; j < row; ++j) {
				if (tmp.index[j*tmp.col + i] != 0) {
					tp = tmp.index[j*tmp.col + i] / tmp.index[i*tmp.col + i];
					for (k = i; k < col; ++k) {
						tmp.index[j*tmp.col + k] -= tp * tmp.index[i*tmp.col + k];
					}
				}
			}
		}
		for (i = 0; i < row; ++i) {
			det *= tmp.index[i*tmp.col + i];
		}
		if (count % 2 == 1)
			det = -det;
		return det;
	}
	//判断是否正定
	bool ifposdef() {
		for (int i = 1; i <= row; ++i) {
			if (block(0, 0, i, i).value() <= 0) {
				return false;
			}
		}
		return true;
	}
	//矩阵修正
	void modify() {
		float b1 = 0, tp;
		for (int i = 0; i < row; ++i) {
			tp = 2 * index[i*col + i];
			for (int j = 0; j < col; ++j) {
				tp -= index[i*col + j];
			}
			b1 = (tp < b1) ? tp : b1;
		}
		for (int i = 0; i < row; ++i) {
			index[i*col + i] -= b1;
		}
	}
	//求伴随矩阵
	Matrix adjoint() {
		Matrix tmp(row, col);
		for (int i = 0; i < tmp.row; ++i) {
			for (int j = 0; j < tmp.col; ++j) {
				if ((i + j) % 2 == 0) {
					tmp.index[i*tmp.col + j] = cofactor(j, i);
				}
				else {
					tmp.index[i*tmp.col + j] = cofactor(j, i) * (-1);
				}
			}
		}
		return tmp;
	}
	//求解线性方程组
	Matrix solve(const Matrix B) {
		float epsilon =1e-9;
		Matrix A(*this);
		Matrix X(B);
		int i, j;
		for (i = 0; i < A.row; ++i) {
			j = i;
			while (j < A.row && fabs(A.index[j*A.col + i]) < epsilon) {
				++j;
			}
			if (j == A.row) {
				printf("Wrong\n");
				A.modify();
				return A.solve(B);
			}
			else {
				if (i != j) {
					A.swaprow(i, j);
					X.swaprow(i, j);
				}
			}
			//将对角位置转换为1
			float tmp = 1.0 / A.index[i*col + i];
			X.rowmultik(i, tmp);
			A.rowmultik(i, tmp);
			//将该列非对角位置转换为0
			for (int k = 0; k < A.row; k++) {
				if (k == i || A.index[k*A.col + i] == 0) {
					continue;
				}
				for (j = i + 1; j < A.col; ++j) {
					A(k, j) -= A(i, j) * A(k, i);
				}
				X(k, 0) -= X(i, 0) * A(k, i);
				A(k, i) = 0;
			}
		}
		return X;
	}
	//求逆矩阵
	Matrix inv() {
		Matrix tmp(*this);
		Matrix eye(row, col);
		eye.E();
		int i, j, k;
		for (i = 0; i < row; ++i) {
			//判断对角方向的元素是否为0
			j = i;
			while (j < row && tmp.index[j*tmp.col + i] == 0) {
				++j;
			}
			if (j == row) {
				cout << "该矩阵不可逆！" << endl;
				return eye;
			}
			else {
				if (i != j) {
					tmp.swaprow(i, j);
					eye.swaprow(i, j);
				}
			}
			//将对角位置转换为1
			float tp = 1.0 / tmp(i, i);
			eye.rowmultik(i, tp);
			tmp.rowmultik(i, tp);
			//将该列非对角位置转换为0
			for (k = 0; k < row; k++) {
				if (k == i || tmp.index[k*tmp.col + i] == 0) {
					continue;
				}
				for (j = i + 1; j < col; j++) {
					tmp.index[k*tmp.col + j] -= tmp(i, j) * tmp(k, i);
				}
				for (j = 0; j < col; j++) {
					eye.index[k*eye.col + j] -= eye(i, j) * tmp(k, i);
				}
				tmp.index[k*tmp.col + i] = 0;
			}
		}
		return eye;
	}
	//数组乘法
	friend Matrix dot(Matrix obj1, Matrix obj2)
	{
		if (obj1.row != obj2.row || obj1.col != obj2.col) {
			cout << "矩阵维数不相等！" << endl;
			return obj1;
		}
		else {
			Matrix tmp(obj1.row, obj1.col);
			for (int i = 0; i < tmp.row*tmp.col; ++i){
				tmp.index[i] = obj1.index[i]*obj2.index[i];
			}
			return tmp;
		}
	}
	//各元素和
	float sum() {
		float tmp = 0;
		for (int i = 0; i < row*col; ++i) {
			tmp += index[i];
		}
		return tmp;
	}
	//各列元素和
	Matrix sumRow() {
		Matrix tmp(1, col);
		tmp.zero();
		for (int i = 0; i < col; ++i) {
			for (int j = 0; j < row; ++j){
				tmp(0, i) += index[j*col + i];
			}
		}
		return tmp;
	}
	//一维范数
	float norm1() {
		float tmp = 0;
		for (int i = 0; i < row*col; ++i) {
			tmp += fabs(index[i]);
		}
		return tmp;
	}
	//二维范数
	float norm2() {
		float tmp = 0;
		for (int i = 0; i < row*col; ++i) {
			tmp += index[i]*index[i];
		}
		return sqrt(tmp);
	}
	//求取两向量夹角
	friend float theta(Matrix obj1, Matrix obj2)
	{
		float tmp = 0;
		for (int i = 0; i < obj1.row*obj1.col; ++i) {
			tmp += obj1.index[i] * obj2.index[i];
		}
		return tmp / obj1.norm2() / obj2.norm2();
	}
};