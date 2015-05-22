#include "Aris_XML.h"
#include <string>
#include <iostream>
#include <functional>

using namespace std;

namespace Aris
{
	namespace Core
	{
		int ReplaceVariable(ELEMENT* pEle)
		{
			/*�ݹ麯������������pEle������Ԫ��*/
			function<void(ELEMENT*, function<void(ELEMENT*)>)>
				recursiveFunc = [&recursiveFunc](ELEMENT* pEle, function<void(ELEMENT*)> pFunc)->void
			{
				if (pFunc != nullptr)
				{
					pFunc(pEle);
				}

				for (ELEMENT* p = pEle->FirstChildElement();
					p != 0;
					p = p->NextSiblingElement())
				{
					recursiveFunc(p, pFunc);
				}
			};
			
			/*���������������滻�ɱ���������*/
			for (ELEMENT* p = pEle->FirstChildElement("Variable")->FirstChildElement();
				p != 0;
				p=p->NextSiblingElement())
			{
				std::string vName, vValue;
				
				vName = string("$(") + string(p->Name()) + string(")");
				vValue = std::string(p->GetText());
				
				recursiveFunc(pEle, [vName,vValue](ELEMENT* p)->void
				{
					const char* text;
					if ((text=p->GetText()) != nullptr)
					{
						string str(text);
						int pos;
						while ((pos = str.find(vName)) != str.npos)
						{
							str.replace(pos, vName.length(), vValue);
						}
						p->SetText(str.c_str());
					}
				});
				
			}
			return 0;
		}

		int str2doubleArray(const char *str, double *doubleArray, unsigned int ArrayLength)
		{
			const char * begin = str;
			char *end;
			
			for (unsigned int i = 0; i < ArrayLength; ++i)
			{
				doubleArray[i] = std::strtod(begin, &end);
				begin = end;
			}
			return 0;
		};
		int doubleArray2str(char *str, const double *doubleArray, unsigned int ArrayLength)
		{
			for (unsigned int i = 0; i < ArrayLength; ++i)
			{
				sprintf(str + i * DOUBLE_SIZE, "%-25.15e", doubleArray[i]);
			}
			
			return 0;
		}
	}
}