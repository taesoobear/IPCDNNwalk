#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
using namespace std;
using namespace Eigen;
void eigenTest()
{
	cout<<"hihi"<<endl;
//https://eigen.tuxfamily.org/dox/classEigen_1_1SparseVector.html
		Eigen::SparseVector<Eigen::Vector3d> j(10);
		std::cout << j<< std::endl;
		j.coeffRef(0)=Vector3d(10,10,10);
		j.coeffRef(7)=Vector3d(10,10,10);
		j.coeffRef(3)=Vector3d(10,10,10);
		j.coeffRef(9)=Vector3d(0);
		j.coeffRef(4)(0)=11;
		j.coeffRef(4)(1)=13;
		for(Eigen::SparseVector<Eigen::Vector3d>::InnerIterator it(j);it; ++it)
		{
			cout<<it.value()<<endl;
			cout<<it.index()<<endl;
		}
int nnz=j.nonZeros();
int* iip=j.innerIndexPtr();
cout<<Eigen::Matrix<int, 1, Eigen::Dynamic>::Map	(iip, nnz)<<endl;
}
/*
 *
 *
 *
 *
 *
 * lklkkp
    inline Scalar& atWithInsertion(Index key, Scalar defaultValue = Scalar(0))
    {
      size_t id = searchLowerIndex(0,m_size,key);
      if (id>=m_size || m_indices[id]!=key)
      {
        resize(m_size+1,1);
        for (size_t j=m_size-1; j>id; --j)
        {
          m_indices[j] = m_indices[j-1];
          m_values[j] = m_values[j-1];
        }
        m_indices[id] = key;
        m_values[id] = defaultValue;
      }
      return m_values[id];
    }
    void prune(Scalar reference, RealScalar epsilon = NumTraits<RealScalar>::dummy_precision())
    {
      size_t k = 0;
      size_t n = size();
      for (size_t i=0; i<n; ++i)
      {
        if (!internal::isMuchSmallerThan(value(i), reference, epsilon))
        {
          value(k) = value(i);
          index(k) = index(i);
          ++k;
        }
      }
      resize(k,0);
    }
    inline Scalar& coeffRef(Index row, Index col)
    {
      eigen_assert((IsColVector ? col : row)==0);
      return coeff(IsColVector ? row : col);
    }
    inline Scalar& coeffRef(Index i)
    {
      return m_data.atWithInsertion(i);
    }
    inline Index searchLowerIndex(size_t start, size_t end, Index key) const
    {
      while(end>start)
      {
        size_t mid = (end+start)>>1;
        if (m_indices[mid]<key)
          start = mid+1;
        else
          end = mid;
      }
      return static_cast<Index>(start);
    }

	*/
