#if !defined(AFX_LAWDLG_H__FBB89F43_B862_44BE_BC9D_17D3D90F4290__INCLUDED_)
#define AFX_LAWDLG_H__FBB89F43_B862_44BE_BC9D_17D3D90F4290__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// LawDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CLawDlg dialog

class CLawDlg : public CDialog
{
// Construction
public:
	CLawDlg(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CLawDlg)
	enum { IDD = IDD_LawDlg };
		// NOTE: the ClassWizard will add data members here
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CLawDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CLawDlg)
		// NOTE: the ClassWizard will add member functions here
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_LAWDLG_H__FBB89F43_B862_44BE_BC9D_17D3D90F4290__INCLUDED_)
