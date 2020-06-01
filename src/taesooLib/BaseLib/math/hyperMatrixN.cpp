#include "stdafx.h"
#include "mathclass.h"
#include "./hyperMatrixN.h"


intmat3D::intmat3D(void)
{
	pages=nrows=columns=0;
	data=NULL;
}
intmat3D::intmat3D(int ll, int mm, int nn)
{
	pages=nrows=columns=0;
	data=NULL;
	setSize(ll,mm,nn);
}

intmat3D::intmat3D(const intmat3D& other)
{
	pages=nrows=columns=0;
	data=NULL;
	assign(other);
}
intmat3D::~intmat3D(void)
{
	for(int i=0; i<pages; i++)
		delete m_pages[i];
	delete [] data;
}

void intmat3D::setSize( int ll, int mm, int nn)
{
	for(int i=0; i<pages; i++)
		delete m_pages[i];

	//!< 원래 데이타 유지 보장 전혀 없음.
	data=new int[ll*mm*nn];

	pages=ll;
	nrows=mm;
	columns=nn;

	m_pages.resize(pages);
	for(int i=0; i<pages; i++)
	{
		m_pages[i]=new intmatrixnView(data+i*mm*nn, mm, nn, nn);
	}
}

intmat3D&  intmat3D::assign(intmat3D const& other)
{
	setSameSize(other);

	for(int i=0; i<page(); i++)
		page(i)=other.page(i);

	return *this;
}


//------------------
hypermatrixn::hypermatrixn(void)
{
	npages=nrows=columns=0;
	data=NULL;
}
hypermatrixn::hypermatrixn(int ll, int mm, int nn)
{
	npages=nrows=columns=0;
	data=NULL;
	setSize(ll,mm,nn);
}

hypermatrixn::hypermatrixn(const hypermatrixn& other)
{
	npages=nrows=columns=0;
	data=NULL;
	assign(other);
}
hypermatrixn::~hypermatrixn(void)
{
	for(int i=0; i<npages; i++)
		delete m_pages[i];
	delete [] data;
}

void hypermatrixn::setSize( int ll, int mm, int nn)
{
	for(int i=0; i<npages; i++)
		delete m_pages[i];

	//!< 원래 데이타 유지 보장 전혀 없음.
	data=new m_real[ll*mm*nn];

	npages=ll;
	nrows=mm;
	columns=nn;

	m_pages.resize(npages);
	for(int i=0; i<npages; i++)
	{
		m_pages[i]=new matrixnView(data+i*mm*nn, mm, nn, nn);
	}
}

hypermatrixn&  hypermatrixn::assign(hypermatrixn const& other)
{
	setSameSize(other);

	for(int i=0; i<page(); i++)
		page(i)=other.page(i);

	return *this;
}

void hypermatrixn::each(const m1::_op& op, const hypermatrixn& other)
{
	for(int i=0; i<page(); i++)
	{
		op.calc(*m_pages[i], other.page(i));
	}
}

/*
hypermatrixn::hypermatrixn(void)
{
	pages=rows=columns=0;m_bDirty=false;
}

hypermatrixn::hypermatrixn(const hypermatrixn& other)
{
	pages=rows=columns=0;m_bDirty=false;
	assign(other);
}

hypermatrixn::hypermatrixn(int ll, int mm, int nn)
{
	pages=rows=columns=0;m_bDirty=false;
	setSize(ll,mm,nn);
}

hypermatrixn::~hypermatrixn(void)
{
	ASSERT(m_nSize==pages);
}

void hypermatrixn::setSize( int ll, int mm, int nn)
{
	Init(ll);
	pages=ll;
	for(int i=0; i<page(); i++)
	{
		page(i).m_pParent=this;
		page(i).setSizeCore(mm,nn);		
	}
	rows=mm;
	columns=nn;
}	

void hypermatrixn::resize(int ll, int mm, int nn)
{
	Resize(ll);
	for(int i=0; i<page(); i++)
	{
		page(i).m_pParent=this;
		page(i).resizeCore(mm,nn);
	}

	rows=mm;
	columns=nn;
}

void hypermatrixn::load(const char* filename, bool bLoadFromBinaryFile)
{
	if(bLoadFromBinaryFile)
	{
		TFile file;
		file.OpenReadFile(filename);
		int npage=file.UnpackInt();
		int nrow=file.UnpackInt();
		int ncolumn=file.UnpackInt();
		setSize(npage, nrow, ncolumn);
		for(int i=0; i<npage; i++)
			file.UnpackArray((*this)[i].buffer, rows()*cols(), sizeof(m_real));
		file.CloseFile();
	}
	else
	{
		ASSERT(0);
	}
}


void hypermatrixn::save(const char* filename, bool bSaveIntoBinaryFile)
{
	if(bSaveIntoBinaryFile)
	{
		TFile file;
		file.OpenWriteFile(filename);
		file.PackInt(page());
		file.PackInt(rows());
		file.PackInt(cols());
		for(int i=0; i<page(); i++)
			file.PackArray((*this)[i].buffer, rows()*cols(), sizeof(m_real));
		file.CloseFile();
	}
	else
	{
		FILE* file=fopen(filename,"w");

		fprintf(file,"page %d row %d col %d\n", page(), rows(), cols());
		for(int pag=0; pag<page(); pag++)
		{
			fprintf(file,"page %d\n", pag);
			for(int i=0; i<rows(); i++)
			{
				for(int j=0; j<cols(); j++)
				{
					fprintf(file,"%f ",(*this)[pag][i][j]);
				}
				fprintf(file,"\n");
			}
		}
		fclose(file);
	}
}

hypermatrixn&  hypermatrixn::op1(const s1::Operator& uop, m_real a)
{
	for(int i=0; i<page(); i++)
		page(i).op1(uop, a);
	return *this;
}

hypermatrixn&  hypermatrixn::op1(const h1::Operator& uop, hypermatrixn const& a)
{
	uop.calc(*this, a);
	return *this;
}

hypermatrixn&  hypermatrixn::op2(const h2::Operator& bop, hypermatrixn const& a, hypermatrixn const& b)
{
	bop.calc(*this, a,b);
	return *this;
}

m_real hypermatrixn::op1(const sh1::Operator& op) const
{
	return op.calc(*this);
}


hypermatrixn&  hypermatrixn::assign(hypermatrixn const& other)
{
	setSameSize(other);

	for(int i=0; i<page(); i++)
		page(i)=other.page(i);

	return *this;
}

bool hypermatrixn::dirty() 
{
	if(!m_bDirty) return false;

	for(int i=0; i<page(); i++)
	{
		if(page(i).rows()!=rows() || page(i).cols()!=cols()) return true;
	}

	m_bDirty=false;
	return false;
}

void hypermatrixn::clean()
{
	// 원소 크기가 다른 경우 안전한 더 큰 크기로 바꾸어준다.
	if(dirty())
	{
		int maxRow=0;
		int maxCol=0;
		for(int i=0; i<page(); i++)
		{
			maxRow=MAX(maxRow, page(i).rows());
			maxCol=MAX(maxCol, page(i).cols());
		}

		if(maxRow!=rows() || maxCol!=cols())
			resize(page(), rows(), cols());
	}
}*/
