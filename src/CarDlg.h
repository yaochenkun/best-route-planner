#if !defined(AFX_CARDLG_H__764BB060_C15E_4329_A214_2120E6959C27__INCLUDED_)
#define AFX_CARDLG_H__764BB060_C15E_4329_A214_2120E6959C27__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// CarDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CCarDlg dialog

class CCarDlg : public CDialog
{
// Construction
public:
	CCarDlg(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CCarDlg)
	enum { IDD = IDD_CarDlg };
	CString	m_KindStr;
	CString	m_OilStr;
	CString	m_PowerStr;
	CString	m_WeightStr;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CCarDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CCarDlg)
	virtual BOOL OnInitDialog();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_CARDLG_H__764BB060_C15E_4329_A214_2120E6959C27__INCLUDED_)
