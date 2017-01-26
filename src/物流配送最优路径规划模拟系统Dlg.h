// 物流配送最优路径规划模拟系统Dlg.h : header file
//

#if !defined(AFX_DLG_H__BCD3097B_3692_47A4_9320_BEB211C62255__INCLUDED_)
#define AFX_DLG_H__BCD3097B_3692_47A4_9320_BEB211C62255__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/////////////////////////////////////////////////////////////////////////////
// CMyDlg dialog

class CMyDlg : public CDialog
{
// Construction
public:
	CMyDlg(CWnd* pParent = NULL);	// standard constructor
	void WriteData();
//关联变量
	CMenu m_Menu;//弹出菜单
	CPen pen1;//路径橙色
	CPen pen2;//地址深蓝
	CPen pen3;//还原浅蓝

//自定义函数
	void   DarkBlue(int x,int y);//画深蓝点
	void   LightBlue(int x,int y);//画浅蓝点
	void   AllBlue();//画所有蓝点
	void   StoreFlag();//仓库旗图样
	void   ClientFlag();//客户旗图样
	void   BeenFlag();//红勾图样
	void   ShowBeen();//筛选已经到过的客户
	void   CutScreen();//截图
	void   PutScreen();//显示截图
	void   ChangeAnytime(int first,int second);//速度与路况随时更新
	void   DrawLine();//画行进红线
	void   Map();//地图图样
	void   Picture(int x1,int y1,int x2,int y2);//道路重绘
	void   OnPicture(int x1,int y1,int x2,int y2);//选中道路
	void   TrafficFlag(int m);//把之前标记了拥堵的道路都重画
	void   Statistic();//计算总里程、总成本、时间、速度、路况
	BOOL   JoinTable(int r);//检测用户双击的道路是否合法
// Dialog Data
	//{{AFX_DATA(CMyDlg)
	enum { IDD = IDD_MY_DIALOG };
	CString	m_DistanceStr;
	int		m_MinRadio;
	CString	m_PrimeStr;
	CString	m_SpeedStr;
	CString	m_TrafficStr;
	CString	m_WholeStr;
	CString	m_FuelEdit;
	CString	m_NeedEdit;
	CString	m_OtherEdit;
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CMyDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	//{{AFX_MSG(CMyDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnOkButton();
	afx_msg void OnStopButton();
	afx_msg void OnReplayButton();
	afx_msg void OnResetButton();
	afx_msg void OnTimer(UINT nIDEvent);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnClientMenu();
	afx_msg void OnStoreMenu();
	afx_msg void OnSmoothMenu();
	afx_msg void OnCrowdMenu();
	afx_msg void OnTestButton();
	afx_msg void OnHelpMenu();
	afx_msg void OnLButtonDblClk(UINT nFlags, CPoint point);
	afx_msg void OnCarMenu();
	afx_msg void OnLawMenu();
	afx_msg void OnQueryMenu();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DLG_H__BCD3097B_3692_47A4_9320_BEB211C62255__INCLUDED_)
