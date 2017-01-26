// HelpDlg.cpp : implementation file
//

#include "stdafx.h"
#include "物流配送最优路径规划模拟系统.h"
#include "HelpDlg.h"
#include "HelpDlg2.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CHelpDlg dialog

CHelpDlg::CHelpDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CHelpDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CHelpDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}


void CHelpDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CHelpDlg)
		// NOTE: the ClassWizard will add DDX and DDV calls here
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CHelpDlg, CDialog)
	//{{AFX_MSG_MAP(CHelpDlg)
	ON_BN_CLICKED(IDC_EnterButton, OnOkButton)
	ON_BN_CLICKED(IDC_NextButton, OnNextButton)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CHelpDlg message handlers

void CHelpDlg::OnNextButton() //下一页按钮
{
	//关闭使用说明1对话框
	OnOK();

	//弹出使用说明2对话框
	CHelpDlg2 dlg;
	dlg.DoModal();
}

void CHelpDlg::OnOkButton() //关闭按钮
{
	// TODO: Add your control notification handler code here
	OnOK();
}