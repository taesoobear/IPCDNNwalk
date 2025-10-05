#pragma once
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "hyperMatrixN.h"
using namespace std;

#define TENSOR_MAX_DIMS 5
template <class T>
class _tensor
{
	protected:
		// Number of dimensions and values
		uint8_t ndims;
		uint32_t nvalues;

		// Number of elements in each dimension
		uint32_t dims[TENSOR_MAX_DIMS + 1];
		uint32_t stride[TENSOR_MAX_DIMS ];

		// Data
		bool _owner;	
		T *data;

		void _init(uint32_t *dims, T* elems=NULL)
		{
			RANGE_ASSERT(data==NULL);
			_owner=true;
			// Check for scalar
			size_t size = 0;
			uint8_t ndims = -1;
			if (dims == NULL || dims[0] == 0)
			{
				this->nvalues= 0;
				this->ndims = 0;
				return;
			}
			else
			{
				while (dims[++ndims] != 0)
				{
					if (size == 0)
					{

						size = dims[ndims];
					}
					else
					{
						size *= dims[ndims];
					}
				}
			}
			stride[ndims-1]=1;
			if(ndims>1)
			{
				for(int i=ndims-2; i>=0; i--)
				{
					stride[i]=stride[i+1]*dims[i+1];
				}
			}

			// Check for limits
			RANGE_ASSERT(size <= UINT32_MAX);

			// If no size, then return NULL
			if (size == 0)
			{
				Msg::error("tensor_dtype_create failed, zero size\n");
			}

			// Return NULL if no dimensions or too many dimensions
			if (ndims > TENSOR_MAX_DIMS)
			{
				Msg::error("tensor_dtype_create failed, ndims=%d\n", ndims);
			}

			// Allocate memory for data and set it
			data= (T*)malloc(sizeof(T)*size);
			if (data == NULL)
			{
				Msg::error("tensor_dtype_create failed, out of memory allocating %ld bytes\n", size * sizeof(T));
			}
			else if (elems != NULL)
			{
				memcpy(data, elems, size * sizeof(T));
			}
			else
			{
				memset(data, 0, size * sizeof(T));
			}

			// Set tensor properties
			this->ndims = ndims;
			this->nvalues = size;
			for (uint8_t i = 0; i < ndims; i++)
			{
				this->dims[i] = dims[i];
			}
			this->dims[ndims] = 0;
		}
		int _prepareSlice_1d(int * indices, const intvectorn& _indices) const {
			int idim=-1;
			for(int i=0; i<_indices.size(); i++)
				indices[i]=_indices[i];

			for(int i=0; i<_indices.size(); i++)
			{
				if(indices[i]==-1)
				{
					idim=i;
					indices[i]=0;
					break;
				}
				else RANGE_ASSERT(indices[i]>=0 && indices[i]<dims[i]);
			}
			RANGE_ASSERT(idim>=0);
			return idim;
		}
		void _prepareSlice(intvectorn const&_indices,intvectorn& newstride, intvectorn& newdims, int* indices) const
		{
			newstride.reserve(TENSOR_MAX_DIMS );
			newdims.reserve(TENSOR_MAX_DIMS );

			for(int i=0; i<_indices.size(); i++)
				indices[i]=_indices[i];

			for(int i=0; i<_indices.size(); i++)
			{
				if(indices[i]==-1)
				{
					newstride.pushBack(stride[i]);
					newdims.pushBack(dims[i]);

					indices[i]=0;
				}
				else RANGE_ASSERT(indices[i]>=0 && indices[i]<dims[i]);
			}
			RANGE_ASSERT(newstride.size()>0);
		}
		template <class T2> void _assign3D(T2& other)
		{
			if(_owner)
			{
				free(data); data=NULL;
				uint32_t dims[]={(uint32_t)other.pages(), (uint32_t)other.rows(),(uint32_t)other.cols(), 0}; _init(dims); 
			}

			RANGE_ASSERT(ndims==3);
			RANGE_ASSERT(dims[0]==other.pages());
			RANGE_ASSERT(dims[1]==other.rows());
			RANGE_ASSERT(dims[2]==other.cols());
			for(int i=0; i<dims[0]; i++)
				for(int j=0; j<dims[1]; j++)
					for(int k=0; k<dims[2]; j++)
						data[i*stride[0]+j*stride[1]+k*stride[2]]=(T)other(i,j,k);
		}
		template <class T2> void _assign(_tvectorn<T2,T2> const& other)
		{
			if(_owner)
			{
				free(data); data=NULL;
				uint32_t dims[]={(uint32_t)other.size(),0}; _init(dims); 
			}

			RANGE_ASSERT(ndims==1);
			RANGE_ASSERT(dims[0]==other.size());
			for(int i=0; i<dims[0]; i++)
				data[i*stride[0]]=(T)other(i);
		}
		template <class T2> void _assign(_tmat<T2> const& other)
		{
			if(_owner)
			{
				free(data); data=NULL;
				uint32_t dims[]={(uint32_t)other.rows(),(uint32_t)other.cols(), 0}; _init(dims); 
			}

			RANGE_ASSERT(ndims==2);
			RANGE_ASSERT(dims[0]==other.rows());
			RANGE_ASSERT(dims[1]==other.cols());
			for(int i=0; i<dims[0]; i++)
				for(int j=0; j<dims[1]; j++)
					data[i*stride[0]+j*stride[1]]=(T)other(i,j);
		}
		template <class T2> void _setMat(_tmat<T2>& temp) const
		{
			for(int i=0; i<dims[0]; i++)
				for(int j=0; j<dims[1]; j++)
					temp(i,j)=(T2)(data[i*stride[0]+j*stride[1]]);
		}
		void _assignRef(const _tensor<T>& other)
		{
			// Set tensor properties
			_owner=false;
			this->ndims = other.ndims;
			for (uint8_t i = 0; i < ndims; i++)
			{
				this->dims[i] = other.dims[i];
				this->stride[i] = other.stride[i];
			}
			this->dims[ndims] = 0;
			this->nvalues = other.nvalues;
			this->data=other.data;
		}
		void _assignRef(T* data, const intvectorn& stride, const intvectorn& dims)
		{
			// Set tensor properties
			_owner=false;
			this->ndims = dims.size();
			for (uint8_t i = 0; i < ndims; i++)
			{
				this->dims[i] = dims[i];
				this->stride[i] = stride[i];
			}
			this->dims[ndims] = 0;

			size_t size=1;
			for(int i=0; i<ndims; i++)
				size *= dims[i];

			this->nvalues = size;
			this->data=data;
		}
	public:
		const uint32_t* strides() const { return stride;}
		T* dataPointer() const { return data;}
		_tensor(uint32_t *dims, T* elems=NULL)
		{
			data=NULL;
			_init(dims, elems);
		}
		_tensor()
		{
			data=NULL;
			_init(NULL,NULL);
		}
		virtual ~_tensor()
		{
			if(_owner) free(data);
		}
		void init(intvectorn const& shape)
		{
			RANGE_ASSERT(_owner);
			if(data) { free(data); data=NULL;}
			switch(shape.size())
			{
				case 0:
					{
					uint32_t dims[]={0}; _init(dims); 
					}
					break;
				case 1:
					{
					uint32_t dims[2]; dims[0]=shape(0); dims[1]=0; _init(dims); 
					}
					break;
				case 2:
					{
					uint32_t dims[3]; dims[0]=shape(0); dims[1]=shape(1); dims[2]= 0; _init(dims); 
					}
					break;
				case 3:
					{
					uint32_t dims[4]; dims[0]=shape(0); dims[1]=shape(1); dims[2]=shape(2); dims[3]= 0; _init(dims); 
					}
					break;
				case 4:
					{
					uint32_t dims[5]; dims[0]=shape(0); dims[1]=shape(1); dims[2]=shape(2); dims[3]=shape(3); dims[4]= 0; _init(dims); 
					}
					break;
			}
			cout<<"nvalues:"<<nvalues<<endl;
		}
		uint32_t ndim() const {return ndims;}
		template <class T2> void _assign(_tensor<T2> const& other)
		{
			RANGE_ASSERT(ndims==other.ndim());
			for(int i=0; i<ndims; i++)
				RANGE_ASSERT(dims[i]==other.shape(i));

			// recursion으로 짤수도 있지만 귀찮음.
			switch(ndims)
			{
				case 1:
					{
						for(int i=0; i<dims[0]; i++)
							data[i*stride[0]]=(T)other.dataPointer()[i*other.strides()[0]];
					}
					break;
				case 2:
					{
						for(int i=0; i<dims[0]; i++)
							for(int j=0; j<dims[1]; j++)
								data[i*stride[0]+j*stride[1]]=(T)other.dataPointer()[i*other.strides()[0]]+j*other.strides()[1];
					}
					break;
				case 3:
					{
						for(int i=0; i<dims[0]; i++)
							for(int j=0; j<dims[1]; j++)
								for(int k=0; k<dims[2]; k++)
									data[i*stride[0]+j*stride[1]+k*stride[2]]=(T)
										other.dataPointer()[i*other.strides()[0]+j*other.strides()[1]+k*other.strides()[2]];
					}
					break;
				case 4:
					{
						for(int i=0; i<dims[0]; i++)
							for(int j=0; j<dims[1]; j++)
								for(int k=0; k<dims[2]; k++)
									for(int l=0; l<dims[3]; l++)
										data[i*stride[0]+j*stride[1]+k*stride[2]+l*stride[3]]=(T)
											other.dataPointer()[i*other.strides()[0]+j*other.strides()[1]+k*other.strides()[2]+l*other.strides()[3]];
					}
					break;
				case 5:
					{
						for(int i=0; i<dims[0]; i++)
							for(int j=0; j<dims[1]; j++)
								for(int k=0; k<dims[2]; k++)
									for(int l=0; l<dims[3]; l++)
										for(int m=0; m<dims[4]; m++)
											data[i*stride[0]+j*stride[1]+k*stride[2]+l*stride[3]+m*stride[4]]=(T)
												other.dataPointer()[i*other.strides()[0]+j*other.strides()[1]+k*other.strides()[2]+l*other.strides()[3]+m*other.strides()[4]];
					}
					break;
				default:
					Msg::error("not implemented yet");
			}
		}
		void setAllValue(T value)
		{
			if(_owner){
				for(int i=0; i<nvalues; i++)
					data[i]=value;
			}
			else
			{
				// recursion으로 짤수도 있지만 귀찮음.
				switch(ndims)
				{
					case 1:
						{
							for(int i=0; i<dims[0]; i++)
								data[i*stride[0]]=value;
						}
						break;
					case 2:
						{
							for(int i=0; i<dims[0]; i++)
								for(int j=0; j<dims[1]; j++)
									data[i*stride[0]+j*stride[1]]=value;
						}
						break;
					case 3:
						{
							for(int i=0; i<dims[0]; i++)
								for(int j=0; j<dims[1]; j++)
									for(int k=0; k<dims[2]; k++)
										data[i*stride[0]+j*stride[1]+k*stride[2]]=value;
						}
						break;
					case 4:
						{
							for(int i=0; i<dims[0]; i++)
								for(int j=0; j<dims[1]; j++)
									for(int k=0; k<dims[2]; k++)
										for(int l=0; l<dims[3]; l++)
											data[i*stride[0]+j*stride[1]+k*stride[2]+l*stride[3]]=value;
						}
						break;
					case 5:
						{
							for(int i=0; i<dims[0]; i++)
								for(int j=0; j<dims[1]; j++)
									for(int k=0; k<dims[2]; k++)
										for(int l=0; l<dims[3]; l++)
											for(int m=0; m<dims[4]; m++)
												data[i*stride[0]+j*stride[1]+k*stride[2]+l*stride[3]+m*stride[4]]=value;
						}
						break;


					default:
						Msg::error("not implemented yet");
				}
			}
		}

		uint32_t shape(int dim) const { return dims[dim];}
		intvectornView shape() const { return intvectornView((const int*)dims, ndims);} 
		unsigned int pages()  const { RANGE_ASSERT(ndims>=1); return dims[0];}
		unsigned int rows() const   { RANGE_ASSERT(ndims>=2); return dims[ndims-2];}
		unsigned int cols() const  { RANGE_ASSERT(ndims>=1); return dims[ndims-1];}

		T get(int i, int j, int k)
		{
			RANGE_ASSERT(ndims==3);
			RANGE_ASSERT(i>=0 && i<dims[0]);
			RANGE_ASSERT(j>=0 && j<dims[1]);
			RANGE_ASSERT(k>=0 && k<dims[2]);
			return data[i*stride[0]+j*stride[1]+k*stride[2]];
		}
		T get(int i, int j, int k, int l)
		{
			RANGE_ASSERT(i>=0 && i<dims[0]);
			RANGE_ASSERT(j>=0 && j<dims[1]);
			RANGE_ASSERT(k>=0 && k<dims[2]);
			RANGE_ASSERT(l>=0 && l<dims[3]);
			RANGE_ASSERT(ndims==4);
			return data[i*stride[0]+j*stride[1]+k*stride[2]+l*stride[3]];
		}

		void set(int i, int j, int k, T f)
		{
			RANGE_ASSERT(ndims==3);
			RANGE_ASSERT(i>=0 && i<dims[0]);
			RANGE_ASSERT(j>=0 && j<dims[1]);
			RANGE_ASSERT(k>=0 && k<dims[2]);
			data[i*stride[0]+j*stride[1]+k*stride[2]]=f;
		}
		void set(int i, int j, int k, int l, T f)
		{
			RANGE_ASSERT(ndims==4);
			RANGE_ASSERT(i>=0 && i<dims[0]);
			RANGE_ASSERT(j>=0 && j<dims[1]);
			RANGE_ASSERT(k>=0 && k<dims[2]);
			RANGE_ASSERT(l>=0 && l<dims[3]);
			data[i*stride[0]+j*stride[1]+k*stride[2]+l*stride[3]]=f;
		}
		T& get_ref(const intvectorn& indices) const
		{
			RANGE_ASSERT(ndims==indices.size());
			int index=0;
			for(int  i=0; i<ndims; i++)
				index+=indices(i)*stride[i];
			RANGE_ASSERT(index>=0 && index<nvalues);
			return data[index];
		}
		inline void set(const intvectorn& indices, T f)
		{
			get_ref(indices)=f;
		}
};

class floatTensorView;
class Tensor;
class floatTensor: public _tensor<float>
{
	public:
		floatTensor()
		{
			uint32_t dims[]={0}; _init(dims); 
		}
		floatTensor(uint32_t i, uint32_t j, uint32_t k)
		{
			uint32_t dims[]={i,j,k,0}; _init(dims); setAllValue(0.f);
		}
		floatTensor(uint32_t i, uint32_t j, uint32_t k, uint32_t l)
		{
			uint32_t dims[]={i,j,k,l,0}; _init(dims); setAllValue(0.f);
		}
		floatTensor(uint32_t i, uint32_t j, uint32_t k, uint32_t l, uint32_t m)
		{
			uint32_t dims[]={i,j,k,l,m,0}; _init(dims); setAllValue(0.f);
		}
		// a[0,1,:] -> a.slice_1d(CT.ivec(0,1,-1))
		floatvecView slice_1d(const intvectorn& _indices) const
		{
			int indices[TENSOR_MAX_DIMS + 1];
			int idim=_prepareSlice_1d(indices, _indices);
			intvectornView ii(indices, _indices.size()); 
			return floatvecView(&get_ref(ii), dims[idim], stride[idim]);
		}

		floatTensorView slice(const intvectorn& _indices) const;
		void assign(floatTensor const& other)
		{
			_assign(other);
		}
		void assign(Tensor const& other);
		void assign(floatvec const& other)
		{
			_assign(other);
		}
		void assign(vectorn const& other)
		{
			_assign(other);
		}
		void assign(matrixn const& other)
		{
			_assign(other);
		}
		void assign(hypermatrixn const& other)
		{
			_assign3D(other);
		}
		matrixn toMat() const
		{
			RANGE_ASSERT(ndims==2);
			matrixn temp(dims[0], dims[1]);
			_setMat(temp);
			return temp;
		}
};

class TensorView;
class Tensor: public _tensor<double>
{
	friend class TensorView;
	public:
		Tensor()
		{
			uint32_t dims[]={0}; _init(dims); 
		}
		Tensor(uint32_t i, uint32_t j, uint32_t k)
		{
			uint32_t dims[]={i,j,k,0}; _init(dims); setAllValue(0.0);
		}
		Tensor(uint32_t i, uint32_t j, uint32_t k, uint32_t l)
		{
			uint32_t dims[]={i,j,k,l,0}; _init(dims); setAllValue(0.0);
		}
		Tensor(uint32_t i, uint32_t j, uint32_t k, uint32_t l, uint32_t m)
		{
			uint32_t dims[]={i,j,k,l,m,0}; _init(dims); setAllValue(0.0);
		}

		// a[0,1,:] -> a.slice_1d(CT.ivec(0,1,-1))
		vectornView slice_1d(const intvectorn& _indices) const
		{
			int indices[TENSOR_MAX_DIMS + 1];
			int idim=_prepareSlice_1d(indices, _indices);
			intvectornView ii(indices, _indices.size()); 
			return vectornView(&get_ref(ii), dims[idim], stride[idim]);
		}

		TensorView page(int i) const;
		TensorView slice(const intvectorn& _indices) const;
		void assign(Tensor const& other)
		{
			_assign(other);
		}
		void assign(floatTensor const& other)
		{
			_assign(other);
		}
		void assign(vectorn const& other)
		{
			_assign(other);
		}
		void assign(floatvec const& other)
		{
			_assign(other);
		}
		void assign(matrixn const& other)
		{
			_assign(other);
		}
		void assign(hypermatrixn const& other)
		{
			_assign3D(other);
		}
		matrixn toMat() const
		{
			RANGE_ASSERT(ndims==2);
			matrixn temp(dims[0], dims[1]);
			_setMat(temp);
			return temp;
		}
		TString shortOutput() const;
};

class floatTensorView: public floatTensor
{
	public:
		floatTensorView(float* data, const intvectorn &stride, const intvectorn &dims)
		{
			_assignRef(data, stride, dims);
		}
	floatTensorView(const floatTensorView& other)				{ 
		_assignRef(other);
	}
	floatTensorView(const floatTensor& other)				{ 
		_assignRef(other);
	}

};
inline floatTensorView floatTensor::slice(const intvectorn& _indices) const
{
	intvectorn newstride;
	intvectorn newdims;
	int indices[TENSOR_MAX_DIMS + 1];

	_prepareSlice(_indices,newstride, newdims, indices);
	intvectornView ii(indices, _indices.size()); 
	return floatTensorView(&get_ref(ii), newstride,newdims);
}
class TensorView: public Tensor
{
	public:
		TensorView(double* data, const intvectorn &stride, const intvectorn &dims)
		{
			_assignRef(data, stride, dims);
		}
	TensorView(const TensorView& other)				{ 
		_assignRef(other);
	}
	TensorView(const Tensor& other)				{ 
		_assignRef(other);
	}
};
inline TensorView Tensor::page(int index) const
{
	int indices[TENSOR_MAX_DIMS ];
	int newdims[TENSOR_MAX_DIMS ];
	int newstride[TENSOR_MAX_DIMS ];
	indices[0]=index;
	for(int i=1; i<ndims; i++)
	{
		indices[i]=0;
		newdims[i-1]=dims[i];
		newstride[i-1]=stride[i];
	}

	intvectornView ii(indices, ndims); 
	intvectornView _newdims(newdims,ndims-1);
	intvectornView _newstride(newstride,ndims-1);
	return TensorView(&get_ref(ii), _newstride,_newdims);
}
inline TString Tensor::shortOutput() const
{
	if(ndims==2)
		return "Tensor (shape="+shape().output()+") :\n"+toMat().shortOutput();
	else if(ndims==1)
		return "Tensor (shape=("+TString("", dims[0])+")) :\n"+slice_1d(intvectorn(1, -1)).shortOutput();
	else
		return "Tensor (shape="+shape().output()+") :\n [0]="+page(0).shortOutput()+TString("...\n [",pages()-1)+TString("]=")+page(pages()-1).shortOutput();
}
inline TensorView Tensor::slice(const intvectorn& _indices) const
{
	intvectorn newstride;
	intvectorn newdims;
	int indices[TENSOR_MAX_DIMS + 1];

	_prepareSlice(_indices,newstride, newdims, indices);
	intvectornView ii(indices, _indices.size()); 
	return TensorView(&get_ref(ii), newstride,newdims);
}
inline void floatTensor::assign(Tensor const& other)
{
	_assign(other);
}
inline TensorView tensorView(const hypermatrixn& v)
{
	intvectorn stride(3);
	intvectorn dims(3);
	dims[0]=v.pages();
	dims[1]=v.rows();
	dims[2]=v.cols();
	stride[0]=v.cols()*v.rows();
	stride[1]=v.cols();
	stride[2]=1;
	return TensorView((double*)&v[0](0,0), stride, dims);
}
