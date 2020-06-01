#pragma once
#include "TypeString.h"
class TWord
{
public:
	TWord(unsigned int word)								{ m_nWord=word;}
	TWord(signed int sword)									{ m_nSWord=sword;}
	TWord(unsigned short lowWord, unsigned short highWord)	{ m_aWord[0]=lowWord; m_aWord[1]=highWord; }
	// string계열의 함수는 정해진 길이보다 길어도 에러는 나지 않지만, 비교자체는 정해진 길이 안에서만 이루어짐에 유의.
	TWord(unsigned short lowWord, const TString& str);	//!< highWord: two character string (len<=2)
	TWord(const char *word);							//!< word: four character string. (len<=4)
	TWord(unsigned char A, unsigned char B, unsigned char C, unsigned char D){ m_aByte[0]=A; m_aByte[1]=B;m_aByte[2]=C;m_aByte[3]=D;}
	~TWord(){}

	inline operator int()	const	{ return m_nSWord;}
	unsigned int word()				{ return m_nWord;}
	unsigned short lowWord()		{ return m_aWord[0];}
	unsigned short highWord()		{ return m_aWord[1];}
	unsigned char byte(int i)		{ return m_aByte[i];}
private:
	union
	{
		unsigned char m_aByte[4];
		unsigned short m_aWord[2];
		unsigned int m_nWord;
		signed int m_nSWord;
	};
};

class TSignedWord
{
public:
	TSignedWord(int word)						{ m_nWord=word;}
	TSignedWord(short lowWord, short highWord)	{ m_aWord[0]=lowWord; m_aWord[1]=highWord; }
	TSignedWord(char A, char B, char C, char D)	{ m_aByte[0]=A; m_aByte[1]=B;m_aByte[2]=C;m_aByte[3]=D;}
	~TSignedWord(){}

	inline operator int()	const	{ return m_nWord;}
	int word()				{ return m_nWord;}
	short lowWord()		{ return m_aWord[0];}
	short highWord()		{ return m_aWord[1];}
	char byte(int i)		{ return m_aByte[i];}
private:
	union
	{
		char m_aByte[4];
		short m_aWord[2];
		int m_nWord;
	};
};

int Hash(const char* string);