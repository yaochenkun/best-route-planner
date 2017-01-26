#if !defined(AFX_HELPDLG2_H__C66DC72E_43E4_4D3D_BEEC_BA7F12EB4525__INCLUDED_)
#define AFX_HELPDLG2_H__C66DC72E_43E4_4D3D_BEEC_BA7F12EB4525__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// HelpDlg2.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CHelpDlg2 dialog

class CHelpDlg2 : public CDialog
{
// Construction
public:
	CHelpDlg2(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CHelpDlg2)
	enum { IDD = IDD_HelpDlg2 };
		// NOTE: the ClassWizard will add data members here
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CHelpDlg2)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CHelpDlg2)
	afx_msg void OnLastButton();
	afx_msg void OnEnterButton2();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_HELPDLG2_H__C66DC72E_43E4_4D3D_BEEC_BA7F12EB4525__INCLUDED_)
