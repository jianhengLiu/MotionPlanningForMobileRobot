# HW_5

## matlab1:QP解minimum snap

![image-20200523222037760](C:\Users\94367\AppData\Roaming\Typora\typora-user-images\image-20200523222037760.png)

## matlab2:闭式解求minimum snap

![image-20200524001852599](HW_5.assets/image-20200524001852599.png)

两种算法结果是一样的，但是闭式解由于参数由实际的物理意义，所以求得的更稳定，并且由于是闭式解，求解会更快

# OOQP，二次规划凸优化求解器

## 1.安装

### 1.1.安装MA27

解压`MA27`，进入相应目录运行行

```bash
./configure
make 
sudo make install
```

### 1.2.安装OOQP

解压`OOQP`
```bash
./configure
make 
sudo make install
```

若出现`ma27`相关报错，应关联`libma27.a`

在`.zshrc`中输入

```bash
MA27LIB=/usr/local/lib/libma27.a
export MA27LIB
```

# OOQP的ros包

## 安装

`https://github.com/ethz-asl/ooqp_eigen_interface`

前提条件，你应该已经安装好了`ma27`和`OOQP`

```bash
git clone -b cmake https://github.com/ethz-asl/ooqp_eigen_interface.git
```

表示克隆分支`cmake`下的版本

### Dependencies

- [OOQP](http://pages.cs.wisc.edu/~swright/ooqp/): Object-oriented software for quadratic programming,
- [Eigen](http://eigen.tuxfamily.org/): Linear algebra library.

### Building and Installing

To build and install the OOQP-Eigen interface:

```bash
mkdir build
cd build
cmake ..
sudo make install
```

### Usage

Add the following to the CMakeLists.txt of your project:

```
find_package(OOQPEI REQUIRED)
```

Then, add `${OOQPEI_INCLUDE_DIRS}` this to the *include_directories* and `${OOQEI_LIBRARIES}` to the *add_libraries*.

示例

```cmake
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

find_package(OOQPEI REQUIRED)

include_directories(include
  	${catkin_INCLUDE_DIRS}
  	${Eigen3_INCLUDE_DIRS}
	${OOQPEI_INCLUDE_DIRS}`
)

add_executable(trajectory_generator_node 
	src/trajectory_generator_node.cpp
	src/trajectory_generator_waypoint.cpp
)

target_link_libraries(trajectory_generator_node
   ${catkin_LIBRARIES} 
		${OOQEI_LIBRARIES}
)
```



### 编译报错

```cmake
Scanning dependencies of target ooqpei
[ 33%] Building CXX object CMakeFiles/ooqpei.dir/src/OoqpEigenInterface.cpp.o
[ 66%] Building CXX object CMakeFiles/ooqpei.dir/src/QuadraticProblemFormulation.cpp.o
In file included from /usr/local/include/Eigen/Core:461:0,
                 from /home/chrisliu/software/ooqp_eigen_interface/include/QuadraticProblemFormulation.hpp:44,
                 from /home/chrisliu/software/ooqp_eigen_interface/src/QuadraticProblemFormulation.cpp:42:
/usr/local/include/Eigen/src/Core/CwiseBinaryOp.h: In instantiation of ‘struct Eigen::internal::traits<Eigen::CwiseBinaryOp<Eigen::internal::scalar_sum_op<double, double>, const Eigen::Product<Eigen::Product<Eigen::Transpose<Eigen::SparseMatrix<double, 1> >, Eigen::DiagonalMatrix<double, -1>, 0>, Eigen::SparseMatrix<double, 1>, 2>, const Eigen::SparseView<Eigen::Matrix<double, -1, -1> > > >’:
/usr/local/include/Eigen/src/Core/EigenBase.h:41:59:   required from ‘struct Eigen::EigenBase<Eigen::CwiseBinaryOp<Eigen::internal::scalar_sum_op<double, double>, const Eigen::Product<Eigen::Product<Eigen::Transpose<Eigen::SparseMatrix<double, 1> >, Eigen::DiagonalMatrix<double, -1>, 0>, Eigen::SparseMatrix<double, 1>, 2>, const Eigen::SparseView<Eigen::Matrix<double, -1, -1> > > >’
/usr/local/include/Eigen/src/SparseCore/SparseMatrixBase.h:26:34:   required from ‘class Eigen::SparseMatrixBase<Eigen::CwiseBinaryOp<Eigen::internal::scalar_sum_op<double, double>, const Eigen::Product<Eigen::Product<Eigen::Transpose<Eigen::SparseMatrix<double, 1> >, Eigen::DiagonalMatrix<double, -1>, 0>, Eigen::SparseMatrix<double, 1>, 2>, const Eigen::SparseView<Eigen::Matrix<double, -1, -1> > > >’
/usr/local/include/Eigen/src/SparseCore/SparseCwiseBinaryOp.h:36:7:   required from ‘class Eigen::CwiseBinaryOpImpl<Eigen::internal::scalar_sum_op<double, double>, const Eigen::Product<Eigen::Product<Eigen::Transpose<Eigen::SparseMatrix<double, 1> >, Eigen::DiagonalMatrix<double, -1>, 0>, Eigen::SparseMatrix<double, 1>, 2>, const Eigen::SparseView<Eigen::Matrix<double, -1, -1> >, Eigen::Sparse>’
/usr/local/include/Eigen/src/Core/CwiseBinaryOp.h:77:7:   required from ‘class Eigen::CwiseBinaryOp<Eigen::internal::scalar_sum_op<double, double>, const Eigen::Product<Eigen::Product<Eigen::Transpose<Eigen::SparseMatrix<double, 1> >, Eigen::DiagonalMatrix<double, -1>, 0>, Eigen::SparseMatrix<double, 1>, 2>, const Eigen::SparseView<Eigen::Matrix<double, -1, -1> > >’
/home/chrisliu/software/ooqp_eigen_interface/src/QuadraticProblemFormulation.cpp:73:65:   required from here
/usr/local/include/Eigen/src/Core/CwiseBinaryOp.h:48:8: error: ambiguous template instantiation for ‘struct Eigen::internal::cwise_promote_storage_order<Eigen::Sparse, Eigen::Sparse, 1, 0>’
   enum {
        ^
In file included from /usr/local/include/Eigen/Core:367:0,
                 from /home/chrisliu/software/ooqp_eigen_interface/include/QuadraticProblemFormulation.hpp:44,
                 from /home/chrisliu/software/ooqp_eigen_interface/src/QuadraticProblemFormulation.cpp:42:
/usr/local/include/Eigen/src/Core/util/XprHelper.h:539:66: note: candidates are: template<class LhsKind, int LhsOrder, int RhsOrder> struct Eigen::internal::cwise_promote_storage_order<LhsKind, Eigen::Sparse, LhsOrder, RhsOrder> [with LhsKind = Eigen::Sparse; int LhsOrder = 1; int RhsOrder = 0]
 template <typename LhsKind, int LhsOrder, int RhsOrder>   struct cwise_promote_
                                                                  ^
/usr/local/include/Eigen/src/Core/util/XprHelper.h:540:66: note:                 template<class RhsKind, int LhsOrder, int RhsOrder> struct Eigen::internal::cwise_promote_storage_order<Eigen::Sparse, RhsKind, LhsOrder, RhsOrder> [with RhsKind = Eigen::Sparse; int LhsOrder = 1; int RhsOrder = 0]
 template <typename RhsKind, int LhsOrder, int RhsOrder>   struct cwise_promote_
                                                                  ^
In file included from /usr/local/include/Eigen/Core:461:0,
                 from /home/chrisliu/software/ooqp_eigen_interface/include/QuadraticProblemFormulation.hpp:44,
                 from /home/chrisliu/software/ooqp_eigen_interface/src/QuadraticProblemFormulation.cpp:42:
/usr/local/include/Eigen/src/Core/CwiseBinaryOp.h:48:8: error: incomplete type ‘Eigen::internal::cwise_promote_storage_order<Eigen::Sparse, Eigen::Sparse, 1, 0>’ used in nested name specifier
   enum {
        ^
In file included from /usr/local/include/Eigen/Core:366:0,
                 from /home/chrisliu/software/ooqp_eigen_interface/include/QuadraticProblemFormulation.hpp:44,
                 from /home/chrisliu/software/ooqp_eigen_interface/src/QuadraticProblemFormulation.cpp:42:
/usr/local/include/Eigen/src/SparseCore/SparseCwiseBinaryOp.h: In instantiation of ‘Eigen::CwiseBinaryOpImpl<BinaryOp, Lhs, Rhs, Eigen::Sparse>::CwiseBinaryOpImpl() [with BinaryOp = Eigen::internal::scalar_sum_op<double, double>; Lhs = const Eigen::Product<Eigen::Product<Eigen::Transpose<Eigen::SparseMatrix<double, 1> >, Eigen::DiagonalMatrix<double, -1>, 0>, Eigen::SparseMatrix<double, 1>, 2>; Rhs = const Eigen::SparseView<Eigen::Matrix<double, -1, -1> >]’:
/usr/local/include/Eigen/src/Core/CwiseBinaryOp.h:105:49:   required from ‘Eigen::CwiseBinaryOp<BinaryOp, Lhs, Rhs>::CwiseBinaryOp(const Lhs&, const Rhs&, const BinaryOp&) [with BinaryOp = Eigen::internal::scalar_sum_op<double, double>; LhsType = const Eigen::Product<Eigen::Product<Eigen::Transpose<Eigen::SparseMatrix<double, 1> >, Eigen::DiagonalMatrix<double, -1>, 0>, Eigen::SparseMatrix<double, 1>, 2>; RhsType = const Eigen::SparseView<Eigen::Matrix<double, -1, -1> >; Eigen::CwiseBinaryOp<BinaryOp, Lhs, Rhs>::Lhs = Eigen::Product<Eigen::Product<Eigen::Transpose<Eigen::SparseMatrix<double, 1> >, Eigen::DiagonalMatrix<double, -1>, 0>, Eigen::SparseMatrix<double, 1>, 2>; Eigen::CwiseBinaryOp<BinaryOp, Lhs, Rhs>::Rhs = Eigen::SparseView<Eigen::Matrix<double, -1, -1> >]’
/usr/local/include/Eigen/src/plugins/CommonCwiseBinaryOps.h:27:1:   required from ‘const Eigen::CwiseBinaryOp<Eigen::internal::scalar_sum_op<typename Eigen::internal::traits<T>::Scalar, typename Eigen::internal::traits<OtherDerived>::Scalar>, const Derived, const OtherDerived> Eigen::SparseMatrixBase<Derived>::operator+(const Eigen::SparseMatrixBase<OtherDerived>&) const [with OtherDerived = Eigen::SparseView<Eigen::Matrix<double, -1, -1> >; Derived = Eigen::Product<Eigen::Product<Eigen::Transpose<Eigen::SparseMatrix<double, 1> >, Eigen::DiagonalMatrix<double, -1>, 0>, Eigen::SparseMatrix<double, 1>, 2>; typename Eigen::internal::traits<OtherDerived>::Scalar = double; typename Eigen::internal::traits<T>::Scalar = double]’
/home/chrisliu/software/ooqp_eigen_interface/src/QuadraticProblemFormulation.cpp:73:65:   required from here
/usr/local/include/Eigen/src/SparseCore/SparseCwiseBinaryOp.h:45:7: error: static assertion failed: THE_STORAGE_ORDER_OF_BOTH_SIDES_MUST_MATCH
       EIGEN_STATIC_ASSERT((
       ^
CMakeFiles/ooqpei.dir/build.make:86: recipe for target 'CMakeFiles/ooqpei.dir/src/QuadraticProblemFormulation.cpp.o' failed
make[2]: *** [CMakeFiles/ooqpei.dir/src/QuadraticProblemFormulation.cpp.o] Error 1
CMakeFiles/Makefile2:67: recipe for target 'CMakeFiles/ooqpei.dir/all' failed
make[1]: *** [CMakeFiles/ooqpei.dir/all] Error 2
Makefile:127: recipe for target 'all' failed
make: *** [all] Error 2


```

根据报错信息找到文件`ooqp_eigen_interface/src/QuadraticProblemFormulation.cpp`

对应的第73行

修改

```cpp
Q_temp = A.transpose() * S * A + W.toDenseMatrix().sparseView();
```

为

```cpp
//    数据存储：Matrix创建的矩阵默认是按列存储(col)，Eigen在处理按列存储的矩阵时会更加高效。如果想修改可以在创建矩阵的时候加入参数
  Q_temp = Eigen::SparseMatrix<double, Eigen::RowMajor>(A.transpose()) * S * A + W.toDenseMatrix().sparseView();

```

感谢

https://stackoverflow.com/questions/46025565/how-to-symmetrize-a-sparse-matrix-in-eigen-c

错误的原因为转置后储存格式变为默认的列储存导致储存格式不一致导致的，我是这么猜测的

## 使用方法

在目录`ooqp_eigen_interface/test`下可以看到对应不同的使用方法

以`OoqpEigenInterface_test.cpp`为例

以下是一个不等式约束的最优化问题的例子

```cpp
TEST(OOQPEITest, InequalityConstraints)
{
  Vector2d solution(2.0 / 3.0, 1.0 + 1.0 / 3.0);

  SparseMatrix<double, Eigen::RowMajor> Q;
  Q.resize(2, 2);
  Q.insert(0, 0) = 1.0;
  Q.insert(0, 1) = -1.0;
  Q.insert(1, 0) = -1.0;
  Q.insert(1, 1) = 2.0;
  VectorXd c(2);
  c << -2.0, -6.0;
  SparseMatrix<double, Eigen::RowMajor> A;
  VectorXd b;
  SparseMatrix<double, Eigen::RowMajor> C;
  C.resize(3, 2);
  C.insert(0, 0) = 1.0;
  C.insert(0, 1) = 1.0;
  C.insert(1, 0) = -1.0;
  C.insert(1, 1) = 2.0;
  C.insert(2, 0) = 2.0;
  C.insert(2, 1) = 1.0;
  VectorXd d(3);
  d << -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max();
  VectorXd f(3);
  f << 2.0, 2.0, 3.0;
  VectorXd l(2);
  l << 0.0, 0.0;
  VectorXd u(2);
  u << 1000.0, 1000.0;
  VectorXd x;

  OoqpEigenInterface::solve(Q, c, A, b, C, d, f, l, u, x);

  expectNear(x, solution, 1e-8, OOQPEI_SOURCE_FILE_POS);
}
```





