// QueryDlg.cpp : implementation file
//

#include "stdafx.h"
#include "物流配送最优路径规划模拟系统.h"
#include "QueryDlg.h"
#include <winsock.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CQueryDlg dialog
CQueryDlg::CQueryDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CQueryDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CQueryDlg)
	routeEdit = _T("");
	//}}AFX_DATA_INIT
}


void CQueryDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CQueryDlg)
	DDX_Control(pDX, IDC_RecordList, recordList);
	DDX_Text(pDX, IDC_RouteEdit, routeEdit);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CQueryDlg, CDialog)
	//{{AFX_MSG_MAP(CQueryDlg)
	ON_NOTIFY(NM_CLICK, IDC_RecordList, OnClickRecordList)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CQueryDlg message handlers

BOOL CQueryDlg::OnInitDialog() 
{
	CDialog::OnInitDialog();
	
	// TODO: Add extra initialization here
		// 列表视图控件风格设置
	recordList.SetExtendedStyle(LVS_EX_FLATSB	//扁平风格显示滚动条
	|LVS_EX_FULLROWSELECT							//允许整行选中
	|LVS_EX_HEADERDRAGDROP							//允许整列拖动
	|LVS_EX_ONECLICKACTIVATE						//单击选中项
	|LVS_EX_GRIDLINES);								//画出网格线
	
	//设置表头
	recordList.InsertColumn(1,"记录号",LVCFMT_CENTER,72,1); //设置姓名列
	recordList.InsertColumn(2,"仓库地点",LVCFMT_CENTER,160,2); //设置性别列
	recordList.InsertColumn(3,"地址数",LVCFMT_CENTER,72,3); //设置语文列
	recordList.InsertColumn(4,"需求量",LVCFMT_CENTER,72,4); //设置英语列
	recordList.InsertColumn(5,"油价",LVCFMT_CENTER,72,5); //设置平均列
	recordList.InsertColumn(6,"其余费用",LVCFMT_CENTER,90,6); //设置平均列
	recordList.InsertColumn(7,"总成本",LVCFMT_CENTER,120,7); //设置平均列
	recordList.InsertColumn(8,"总里程",LVCFMT_CENTER,120,8); //设置平均列
	recordList.InsertColumn(9,"预期时长",LVCFMT_CENTER,120,9); //设置平均列


	//载入数据库数据
	ReadData();

	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

void CQueryDlg::ReadData()
{

}

//选中列表的行
void CQueryDlg::OnClickRecordList(NMHDR* pNMHDR, LRESULT* pResult) 
{

}
