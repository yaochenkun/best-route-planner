// CarDlg.cpp : implementation file
//

#include "stdafx.h"
#include "物流配送最优路径规划模拟系统.h"
#include "CarDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CCarDlg dialog


CCarDlg::CCarDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CCarDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CCarDlg)
	m_KindStr = _T("");
	m_OilStr = _T("");
	m_PowerStr = _T("");
	m_WeightStr = _T("");
	//}}AFX_DATA_INIT
}


void CCarDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CCarDlg)
	DDX_Text(pDX, IDC_KindStr, m_KindStr);
	DDX_Text(pDX, IDC_OilStr, m_OilStr);
	DDX_Text(pDX, IDC_PowerStr, m_PowerStr);
	DDX_Text(pDX, IDC_WeightStr, m_WeightStr);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CCarDlg, CDialog)
	//{{AFX_MSG_MAP(CCarDlg)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CCarDlg message handlers

BOOL CCarDlg::OnInitDialog() 
{
	CDialog::OnInitDialog();
	
	FILE *rp=fopen("CarA.txt","r");
	double p,g,o;
	fscanf(rp,"%lf %lf %lf ",&p,&g,&o);
	
	switch((int)g)
	{
		case 2:m_KindStr="小型";break;
		case 5:m_KindStr="中型";break;
		case 8:m_KindStr="大型";break;
	}
	m_PowerStr.Format("%d",(int)p);
	m_WeightStr.Format("%d",(int)g);
	m_OilStr.Format("%.2lf",o);

	UpdateData(FALSE);

	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}
