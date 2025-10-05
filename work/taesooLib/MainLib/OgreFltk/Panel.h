
#ifndef PANEL_H_
#define PANEL_H_
class Panel
{
	CImage* pImage;
	CImagePixel cip;
	CImage icolormap;
	CImagePixel colormap;
	intvectorn colormapIndex;
public:
	Panel():pImage(NULL){}
	~Panel(){}

	bool isCreated()	{ return pImage!=NULL;}
	CImage* ptr()		{ return pImage;}
	// maxValue는 colormap을 0 부터 255의 숫자를 이용해서 access하고 싶으면 255로 준다.
	void create(CImage* pNewImage, int numFrame, int height, int maxValue, const char* colormapfile)
	{
		pImage=pNewImage;
		pImage->Create(numFrame, height);//, 24);

		cip.Init(pImage);

		icolormap.Load(colormapfile);
		colormap.Init(&icolormap);
		setMaxValue(maxValue);
	}

	int currMaxValue()
	{
		return colormapIndex.size()-1;
	}

	void setMaxValue(int maxValue);
	void createPanel(CImage *pNewImage, int numFrame, int maxLen, int maxValue, const char* colormapfile);
	void drawSegmentText(int start, int end, int value, TString const & text);
	void drawBox(int start, int end, int colormapValue);
	void drawTextBox(int start, int end, int colormapValue, const char* text);
	void clear(int start, int end);

};

#endif
