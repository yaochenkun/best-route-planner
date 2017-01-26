// LawDlg.cpp : implementation file
//

#include "stdafx.h"
#include "物流配送最优路径规划模拟系统.h"
#include "LawDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CLawDlg dialog


CLawDlg::CLawDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CLawDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CLawDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}


void CLawDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CLawDlg)
		// NOTE: the ClassWizard will add DDX and DDV calls here
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CLawDlg, CDialog)
	//{{AFX_MSG_MAP(CLawDlg)
		// NOTE: the ClassWizard will add message map macros here
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CLawDlg message handlers
