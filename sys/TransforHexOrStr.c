/*
********************************************************************************
                                16进制字符串相互转换

文件名    ：TransforHexOrStr.c
版本      ：
程序员    ：TK_CG
********************************************************************************
说明：
********************************************************************************
*/


void HexToStr(BYTE *pbDest, uint8 *pbSrc, int nLen)
{
  char ddl,ddh;
  int i;
  
  for (i=0; i<nLen; i++)
  {
    ddh = 48 + pbSrc[i] / 16;
    ddl = 48 + pbSrc[i] % 16;
    if (ddh > 57) ddh = ddh + 7;
    if (ddl > 57) ddl = ddl + 7;
    pbDest[i*2] = ddh;
    pbDest[i*2+1] = ddl;
  }
  
  pbDest[nLen*2] = '\0';
}


void StrToHex(BYTE *pbDest, BYTE *pbSrc, int nLen)
{
  char h1,h2;
  BYTE s1,s2;
  int i;
  
  for (i=0; i<nLen; i++)
  {
    h1 = pbSrc[2*i];
    h2 = pbSrc[2*i+1];
    
    s1 = toupper(h1) - 0x30;
    if (s1 > 9) 
      s1 -= 7;
    
    s2 = toupper(h2) - 0x30;
    if (s2 > 9) 
      s2 -= 7;
    
    pbDest[i] = s1*16 + s2;
  }
}