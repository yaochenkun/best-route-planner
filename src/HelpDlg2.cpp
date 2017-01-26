// HelpDlg2.cpp : implementation file
//

#include "stdafx.h"
#include "物流配送最优路径规划模拟系统.h"
#include "HelpDlg2.h"
#include "HelpDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CHelpDlg2 dialog


CHelpDlg2::CHelpDlg2(CWnd* pParent /*=NULL*/)
	: CDialog(CHelpDlg2::IDD, pParent)
{
	//{{AFX_DATA_INIT(CHelpDlg2)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}


void CHelpDlg2::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CHelpDlg2)
		// NOTE: the ClassWizard will add DDX and DDV calls here
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CHelpDlg2, CDialog)
	//{{AFX_MSG_MAP(CHelpDlg2)
	ON_BN_CLICKED(IDC_LastButton, OnLastButton)
	ON_BN_CLICKED(IDC_EnterButton2, OnEnterButton2)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CHelpDlg2 message handlers

void CHelpDlg2::OnLastButton() //上一页按钮
{
	//关闭使用说明2对话框
	OnOK();

	//弹出使用说明1对话框
	CHelpDlg dlg;
	dlg.DoModal();	
}

void CHelpDlg2::OnEnterButton2() //关闭按钮 
{
	OnOK();	
}
