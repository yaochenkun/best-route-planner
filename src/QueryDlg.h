#if !defined(AFX_QUERYDLG_H__0015FDFD_AE32_442D_9402_F75F6535B4C3__INCLUDED_)
#define AFX_QUERYDLG_H__0015FDFD_AE32_442D_9402_F75F6535B4C3__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// QueryDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CQueryDlg dialog

class CQueryDlg : public CDialog
{
// Construction
public:
	CQueryDlg(CWnd* pParent = NULL);   // standard constructor
	void ReadData();
// Dialog Data
	//{{AFX_DATA(CQueryDlg)
	enum { IDD = IDD_QueryDlg };
	CListCtrl	recordList;
	CString	routeEdit;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CQueryDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CQueryDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnClickRecordList(NMHDR* pNMHDR, LRESULT* pResult);
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_QUERYDLG_H__0015FDFD_AE32_442D_9402_F75F6535B4C3__INCLUDED_)
