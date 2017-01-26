// 物流配送最优路径规划模拟系统Dlg.cpp : implementation file
//

/***************************************************************************************************************************************/
/*头文件*/
/***************************************************************************************************************************************/
#include "stdafx.h"
#include "物流配送最优路径规划模拟系统.h"
#include "物流配送最优路径规划模拟系统Dlg.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <mmsystem.h>
#include "HelpDlg.h"
#include "CarDlg.h"
#include "LawDlg.h"
#include "QueryDlg.h"

#include <winsock.h>
#pragma  comment(lib,"winmm.lib")

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif
/***************************************************************************************************************************************/
/*宏定义*/
/***************************************************************************************************************************************/
#define UNKNOWN    0x0000 //未知

#define OUTDOOR    0x1000 //未选中
#define ONBLUE     0x2000 //选中蓝点
#define ONROAD     0x3000 //选中道路

#define STOREFLAG  0x4000 //标记仓库
#define CLIENTFLAG 0x5000 //标记客户
#define HIDEFLAG   0x6000 //暂时隐藏标记的客户，避免重复查找

#define SMOOTH     0x7000 //道路流畅
#define CROWD      0x8000 //道路拥挤
/***************************************************************************************************************************************/
/*类定义*/
/***************************************************************************************************************************************/
//结点类
typedef struct
{
	int  x; //结点横坐标
	int  y; //结点纵坐标
	int  aroundid[9]; //结点相连结点ID（最多8个结点+结束标志0）
	int  roadid[6]; //结点相连道路ID（最多5条路+结束标志0）
	int  id; //结点ID
	int  choose; //用户的选择（STOREFLAG/CLIENTFLAG）
	int  father; //父结点ID（用于goback()回溯路线）
	BOOL running; //结点所在道路是否正在行进
	BOOL been; //是否已配送到
}NODE;

//道路类
typedef struct
{
	int  id; //道路ID
	int  traffic; //道路路况
	BOOL running; //是否正在行进中
	BOOL then; //路况是否动态添加
}ROAD;

//ASTAR链表类（用于A星算法）
typedef struct astar
{
	int     id; //结点ID
	int     center; //中心结点ID
	double  g; //从仓库起到中心结点扩展出的子结点的路程个g(n)
	double  price; //总路程f(n)=g(n)+h(n) （其中h(n)是启发函数）
	struct  astar *next;
}ASTAR;

//ROUTE链表类（用于储存结点ID）
typedef struct route
{
	int     id; //结点ID
	struct  route *next;
}ROUTE;
/***************************************************************************************************************************************/
/*全局变量*/
/***************************************************************************************************************************************/
//最终路线用
int* finalArray;
ROUTE  *Head=NULL; //头结点
ROUTE  *Start; //开辟用
ROUTE  *Link,*link,*lin; //连接用

CPoint Point1,Point2; //点击坐标、各种图样位置
CDC    memDC1,*pDC1;CBitmap bmp1; CRect rt1; //截图用
BOOL   timer=FALSE,ing=FALSE,end=FALSE,crowd=TRUE,adjust=FALSE,test=FALSE,oncrowd=TRUE; //定时器是否开启、是否在行进中、是否到达终点、结点是否拥挤、是否动态规划、是否点击测试
BOOL   fok=TRUE,fstop=TRUE,freplay=TRUE; //初次点击按钮
int    lastnode; //测试时，上一次所到结点
int    touch=OUTDOOR,only=0,plan=-1,speed=15,mode; //选中状态、仓库唯一、方案、行进速度
double xi,yi,sonx,sony; //当前结点坐标、下一个结点坐标
double D,O,P,M,V,S,T; //总里程(km)、每公里耗油(公里/升)、额定功率(kw)、总质量(t)、速度(m/s)、成本(元)、时间(h)
/***************************************************************************************************************************************/
/*函数声明*/
/***************************************************************************************************************************************/
int  * Array(); //由标注信息动态分配数组
int  * Product(int *order,int n); //将配送顺序变异后产生新顺序
ROUTE* FindRoute(int *store,int *client); //产生以*store和*client为起始点的最短路线链表
ROUTE* FindCircle(int *order); //以order数组里存放的顺序产生巡回路线链表
ROUTE* Cool(int *order,int num); //退火算法求最终巡回路线链表
ROUTE* Recool(ROUTE *old,int times); //检测是否比文件中的旧路线还要短
BOOL   CheckBlank(CString str); //检测填入信息是否为空
BOOL   CheckNum(CString str); //检测数字是否非法
BOOL   CheckSurvive(int one); //检测是否无路可走
BOOL   CheckTraffic(int one,int two); //检测两个结点共有道路是否拥堵
BOOL   CheckNode(int first,int second); //检测first结点周围是否存在second结点
BOOL   CheckCool(ROUTE *left); //检测是否需要对剩余数组重新退火
BOOL   CheckClient(int first,int second); //查找first点和second点之间是否存在未配送到的客户结点
BOOL   CheckOncrowd(int r); //检测客户结点所在道路是否拥堵
void   ChangeCircle(); //动态规划新路线
void   Goback(int son); //以son为最终客户开始回溯路线 
void   FreeAstar(ASTAR *h); //释放ASTAR链表
void   FreeRoute(ROUTE *h); //释放ROUTE链表
void   AddRunning(int first,int second); //由first和second结点确定道路并将其修改为行进中
void   ChooseA(double need);//确定常量
void   FprintfA(double G); //常量写入文件
void   CalculateTime(); //计算时间
void   CalculateHead();//计算Head链表总路程
double CalculateDis(int first,int second); //计算两结点的距离
double CalculateSpeed(int first,int second); //计算速度
/***************************************************************************************************************************************/
/*录入所有结点信息*/
/***************************************************************************************************************************************/
//定义75条道路
ROAD road[76];

//定义71个结点
NODE node[72]=		
//普通结点信息录入
{{0,0},{222,271,{2,5,51,52,0},{1,2,5,0}},{222,333,{1,3,7,52,53,0},{2,3,7,0}},{222,416,{2,4,8,53,54,0},{3,4,8,0}},{222,494,{3,9,54,0},{4,9,0}},{308,271,{1,6,0},{5,6,0}},
{340,280,{5,7,10,55,0},{6,10,11,14,0}},{340,333,{2,6,8,11,0},{7,11,12,15,0}},{340,414,{3,7,9,13,56,0},{8,12,13,16,0}},{340,494,{4,8,14,56,0},{9,13,17,0}},{384,280,{6,11,16,0},{14,18,21,0}},
{384,333,{7,10,12,0},{15,18,22,0}},{426,332,{11,13,17,0},{19,22,23,0}},{426,413,{8,12,14,18,0},{16,19,20,24,0}},{426,493,{9,13,19,0},{17,20,25,0}},{475,218,{16,22,0},{26,30,0}},
{475,277,{10,15,17,24,0},{21,26,27,31,0}},{475,330,{12,16,18,24,25,57,0},{23,27,28,33,34,0}},{475,413,{13,17,19,26,57,62,0},{24,28,29,35,0}},{475,492,{14,18,33,62,0},{25,29,36,0}},
{505,68,{21,27,58,59,0},{37,38,43,0}},{505,123,{20,22,28,59,0},{38,39,44,0}},{505,218,{15,21,23,49,60,0},{30,39,40,45,0}},{505,265,{22,24,25,49,61,0},{32,40,41,75,0}},
{493,285,{16,17,23,0},{31,32,33,0}},{505,330,{17,23,26,31,61,0},{34,41,42,46,0}},{505,413,{18,25,32,0},{35,42,47,0}},{538,68,{20,28,0},{43,48,0}},{538,123,{21,27,29,63,0},{44,48,49,0}},
{538,218,{28,30,49,63,0},{49,50,74,0}},{538,286,{29,31,0},{50,51,0}},{538,330,{25,30,32,36,64,65,0},{46,51,52,55,0}},{538,413,{26,31,33,34,64,65,66,67,0},{47,52,53,56,0}},
{538,491,{19,32,35,66,67,68,0},{36,53,54,58,0}},{584,412,{32,39,0},{56,57,0}},{595,506,{33,42,70,0},{58,59,0}},{612,328,{31,37,0},{55,60,0}},{631,363,{36,38,0},{60,61,0}},
{651,385,{37,39,43,0},{61,62,66,0}},{649,424,{34,38,40,69,0},{57,62,63,67,0}},{648,456,{39,41,0},{63,64,0}},{645,469,{40,42,0},{64,65,0}},{642,508,{35,41,48,70,0},{59,65,68}},
{710,408,{38,44,0},{66,71,0}},{723,432,{43,45,71,0},{71,72,0}},{721,478,{44,46,71,0},{72,73,0}},{713,530,{45,47,0},{70,73,0}},{701,515,{46,48,0},{69,70,0}},{678,509,{42,47,0},{68,69,0}},{529,218,{22,23,29,60,0},{45,74,75,0}},{0,0},
//客户结点信息录入
{222,231,{1,0},{1,0}},{222,302,{1,2,0},{2,0}},{222,382,{2,3,0},{3,0}},{222,463,{3,4,0},{4,0}},{340,228,{6,0},{10,0}},
{340,465,{8,9,0},{13,0}},{475,378,{17,18,0},{28,0}},{505,40,{20,0},{37,0}},{505,100,{20,21,0},{38,0}},{518,218,{22,49,0},{45,0}},
{505,295,{23,25,0},{41,0}},{475,459,{18,19,0},{29,0}},{538,165,{28,29,0},{49,0}},{538,364,{31,32,65,0},{52,0}},{538,395,{31,32,64,0},{52,0}},
{538,437,{32,33,67,0},{53,0}},{538,475,{32,33,66,0},{53,0}},{538,596,{33,0},{54,0}},{665,427,{39,0},{67,0}},{631,508,{35,42,0},{59,0}},{723,453,{44,45,0},{72,0}}};

//客户地点的名称
char* nodename[22]=
{"","丁家屯村","周官屯村","金凯锐商务中心","唐山市工人医院","河北唐山一中","唐山中西医骨科医院","雅园商务中心","建丰百货市场","唐山学院","华岩园","唐山市第九医院","同济医院","唐山宾馆","唐山市政府","唐山饭店","大钊公园","南湖大酒店","南湖公园","唐山市路北区政府","唐山平安医院","唐山肛肠医院"};
/***************************************************************************************************************************************/
class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	//{{AFX_DATA(CAboutDlg)
	enum { IDD = IDD_ABOUTBOX };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CAboutDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	//{{AFX_MSG(CAboutDlg)
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
	//{{AFX_DATA_INIT(CAboutDlg)
	//}}AFX_DATA_INIT
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CAboutDlg)
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
	//{{AFX_MSG_MAP(CAboutDlg)
		// No message handlers
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CMyDlg dialog

CMyDlg::CMyDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CMyDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CMyDlg)
	m_DistanceStr = _T("");
	m_MinRadio = -1;
	m_PrimeStr = _T("");
	m_SpeedStr = _T("");
	m_TrafficStr = _T("");
	m_WholeStr = _T("");
	m_FuelEdit = _T("");
	m_NeedEdit = _T("");
	m_OtherEdit = _T("");
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CMyDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CMyDlg)
	DDX_Text(pDX, IDC_DistanceStr, m_DistanceStr);
	DDX_Radio(pDX, IDC_MindisRadio, m_MinRadio);
	DDX_Text(pDX, IDC_PrimeStr, m_PrimeStr);
	DDX_Text(pDX, IDC_SpeedStr, m_SpeedStr);
	DDX_Text(pDX, IDC_TrafficStr, m_TrafficStr);
	DDX_Text(pDX, IDC_WholeStr, m_WholeStr);
	DDX_Text(pDX, IDC_FuelEdit, m_FuelEdit);
	DDX_Text(pDX, IDC_NeedEdit, m_NeedEdit);
	DDX_Text(pDX, IDC_OtherEdit, m_OtherEdit);
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CMyDlg, CDialog)
	//{{AFX_MSG_MAP(CMyDlg)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_RBUTTONDOWN()
	ON_BN_CLICKED(IDC_OkButton, OnOkButton)
	ON_BN_CLICKED(IDC_StopButton, OnStopButton)
	ON_BN_CLICKED(IDC_ReplayButton, OnReplayButton)
	ON_BN_CLICKED(IDC_ResetButton, OnResetButton)
	ON_WM_TIMER()
	ON_WM_MOUSEMOVE()
	ON_COMMAND(ID_ClientMenu, OnClientMenu)
	ON_COMMAND(ID_StoreMenu, OnStoreMenu)
	ON_COMMAND(ID_SmoothMenu, OnSmoothMenu)
	ON_COMMAND(ID_CrowdMenu, OnCrowdMenu)
	ON_BN_CLICKED(IDC_TestButton, OnTestButton)
	ON_COMMAND(ID_HelpMenu, OnHelpMenu)
	ON_WM_LBUTTONDBLCLK()
	ON_COMMAND(ID_CarMenu, OnCarMenu)
	ON_COMMAND(ID_LawMenu, OnLawMenu)
	ON_COMMAND(ID_QueryMenu, OnQueryMenu)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CMyDlg message handlers

BOOL CMyDlg::OnInitDialog() //初始化函数
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

/*************************************************/
/*初始化设置*/
/*************************************************/	

	//删除记录文件
	remove("mindistance.txt");
	remove("CarA.txt");
	remove("adjust.txt");

	//询问选用何种搜索方式
	if(MessageBox("请问您是否选用“收敛重找模式”？\n收敛重找模式：重新搜索得到的路径一定比上一次的短，容易逼近最短路径。（适合实际用）\n非收敛重找模式：重新搜索的路径是经过一次随机算法得到的不稳定值，用于测试算法的准确性。（适合测试用）",
				  "询问",MB_YESNO|MB_ICONQUESTION)==IDYES)
	{
		mode=0;
		GetDlgItem(IDC_ModeStr)->SetWindowText("收敛重找模式");
	}
	else
	{
		mode=1;
		GetDlgItem(IDC_ModeStr)->SetWindowText("非收敛重找模式");
	}

	//弹出式菜单引入
	m_Menu.LoadMenu(IDR_MarkMenu);
	pen1.CreatePen(PS_SOLID,4,RGB(230,0,0)); //创建画笔对象1（画红色线）
	pen2.CreatePen(PS_SOLID,10,RGB(0,64,152)); //创建画笔对象2（画深蓝点）
	pen3.CreatePen(PS_SOLID,10,RGB(34,174,230)); //创建画笔对象3（画浅蓝点）

	GetDlgItem(IDC_StopButton)->EnableWindow(FALSE);
	GetDlgItem(IDC_ReplayButton)->EnableWindow(FALSE);

	//截图准备
	pDC1=GetWindowDC(); 
	memDC1.CreateCompatibleDC(pDC1)  ; 
	GetWindowRect(&rt1);
	bmp1.CreateCompatibleBitmap(pDC1,rt1.Width(),rt1.Height());  
	memDC1.SelectObject(&bmp1); 

	//初始化结点的其余信息
	for(int i=1;i<=71;i++)
	{
		node[i].id=i; //该结点ID
		node[i].choose=UNKNOWN; //未选择地址
		node[i].running=FALSE; //结点未占用
		node[i].father=UNKNOWN; //指向未知的父结点
		node[i].been=FALSE; //未配送到
	}	
	//初始化道路的其余信息
	for(i=1;i<=75;i++)
	{
		road[i].id=i; //该道路ID
		road[i].traffic=SMOOTH; //该道路路况
		road[i].running=FALSE; //道路可用
		road[i].then=FALSE; //路况非动态添加
	}
	return TRUE;  // return TRUE  unless you set the focus to a control
}
/***************************************************************************************************************************************/
void CMyDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CMyDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CMyDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}


/***************************************************************************************************************************************/
/*消息响应函数定义*/
/***************************************************************************************************************************************/
void CMyDlg::OnOkButton() //确定按钮
{

/************************************/
/*审核输入信息*/
/************************************/
	UpdateData(TRUE); //获取内容
	//需求量
	if(CheckBlank(m_NeedEdit) == FALSE) //是否为空
	{
		MessageBox("您还未输入需求量!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	if(CheckNum(m_NeedEdit) == FALSE) //数字是否非法
	{
		MessageBox("请输入有效需求量!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	if(atof(m_NeedEdit) > 12) //吨数是否超过
	{
		MessageBox("最大载重不超过12吨!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	//燃油费
	if(CheckBlank(m_FuelEdit) == FALSE)
	{
		MessageBox("您还未输入燃油费!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	if(CheckNum(m_FuelEdit) == FALSE)
	{
		MessageBox("请输入有效燃油费!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	if(atof(m_FuelEdit) > 10) //油费是否超过
	{
		MessageBox("燃油费不超过10元/升!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	//其余费
	if(CheckBlank(m_OtherEdit) == FALSE)
	{
		MessageBox("您还未输入其余费!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	if(CheckNum(m_OtherEdit) == FALSE)
	{
		MessageBox("请输入有效其余费!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	//配送方案
	if(m_MinRadio == -1)//判断选中的单选项是（未选中-1,选中第一项0,选中第二项1）
	{
		MessageBox("您还未选择配送方案!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	plan=m_MinRadio;//把方案传给全局变量plan，用来在其他函数里判断是哪个方案	
	//检测用户有没有在图上标记
	int havestore=UNKNOWN,haveclient=UNKNOWN;
	for(int n=51;n<=71;n++)
	{
		switch(node[n].choose)
		{
			case STOREFLAG: havestore=STOREFLAG  ;break;
			case CLIENTFLAG:haveclient=CLIENTFLAG;break;
		}
		if(havestore == STOREFLAG && haveclient == CLIENTFLAG)
			break;
	}
/************************************/
/*审核通过*/
/************************************/
	if(havestore == STOREFLAG && haveclient == CLIENTFLAG)
	{
		ing=TRUE; //标记已经在进行游戏中了

		//按钮提示
		if(fok == TRUE)
		{
			MessageBox("系统已经为您找到了初次最短路径。\n如果结果不满意，您可以多次点击重找按钮更快找到最短路径。","提示",MB_OK|MB_ICONINFORMATION);
			fok=FALSE;
		}
		GetDlgItem(IDC_OkButton)->EnableWindow(FALSE); //控件可选转换
		CutScreen();//截图
	
		int *order=Array();

//此时order[]里存放的就是初次顺序

/************************************/
/*申请最终路线链表并赋予最终路线*/
/************************************/
		
		//初始化起点及其后继结点
		Head=(ROUTE*)malloc(sizeof(ROUTE));
		Start=(ROUTE*)malloc(sizeof(ROUTE));
		Head->id=0;Head->next=Start;
		Start->id=*(order+1);Start->next=NULL;
		Link=Start;

		//退火算法找最终巡回路线
		for(int num=4;*(order+num)!=0;num++); //找数组长度
		Head=Cool(order,++num);
		free(order);order=NULL; //释放数组内存

//此时Head指向最终路线链表，此链表依次存有先后结点ID，Head->id存放该线路的总长
		
/************************************/
/*链表转换成图像显示*/
/************************************/	
		
		int  first,second;//查询行进中的前、次结点
		
		//找前、次结点
		Link=Head->next;
		first=Link->id;node[first].running=TRUE;//前结点占用
		Link=Link->next;
		second=Link->id;node[second].running=TRUE;//次结点占用
		AddRunning(first,second);//由前次结点确定道路是否在行进中

		//赋予前、次结点的坐标
		xi=node[first].x;
		yi=node[first].y;
		sonx=node[second].x;
		sony=node[second].y;

/************************************/
/*文本信息计算与显示*/
/************************************/
		
		//处理控件状态
		GetDlgItem(IDC_NeedEdit)->EnableWindow(FALSE);
		GetDlgItem(IDC_FuelEdit)->EnableWindow(FALSE);
		GetDlgItem(IDC_OtherEdit)->EnableWindow(FALSE);
		GetDlgItem(IDC_MindisRadio)->EnableWindow(FALSE);
		GetDlgItem(IDC_MintimRadio)->EnableWindow(FALSE);
		GetDlgItem(IDC_StopButton)->EnableWindow(TRUE);
		GetDlgItem(IDC_ReplayButton)->EnableWindow(TRUE);
		GetDlgItem(IDC_ResetButton)->EnableWindow(TRUE);	
		
		//状态区显示
		ChooseA(atof(m_NeedEdit)); //确定P、O、M三常量
		V=CalculateSpeed(first,second); //计算速度
		Statistic(); //计算其余量并显示

		timer=TRUE;SetTimer(1,speed,NULL); //标记定时器并开启


		//将该条记录写入数据库
		WriteData();
	}
	else
	{
		MessageBox("您还未标记仓库及客户地址!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
}
void CMyDlg::WriteData()
{

}
/***************************************************************************************************************************************/
void CMyDlg::OnStopButton() //暂停按钮
{
	CString str;
	GetDlgItem(IDC_StopButton)->GetWindowText(str);
	if(str == "启动")
	{
		timer=TRUE;
		SetTimer(1,speed,NULL);
		GetDlgItem(IDC_StopButton)->SetWindowText("暂停");
	}
	else
	{
		timer=FALSE;
		KillTimer(1);

		//按钮提示
		if(fstop == TRUE)
		{
			MessageBox("点击暂停按钮可以停止小车行进，在此期间您可以标记堵车路段。\n当标记完毕后，可以点击启动按钮继续行进。","提示",MB_OK|MB_ICONINFORMATION);
			fstop=FALSE;
		}
		GetDlgItem(IDC_StopButton)->SetWindowText("启动");
	}
}
/***************************************************************************************************************************************/
void CMyDlg::OnReplayButton() //重找按钮
{
	//中断计时器
	if(timer == TRUE)
	{ 
		timer=FALSE;
		KillTimer(1);
	}
/************************************/
/*图像处理*/
/************************************/
	if(freplay == TRUE) //第一次点重找按钮
	{
		MessageBox("多次点击重找按钮可以帮助您更快找到最短路径。\n此外，您还可以利用该按钮重新演示找到的最短路径。","提示",MB_OK|MB_ICONINFORMATION);
		freplay=FALSE; //已经不是第一次点重找按钮了
	}
	PutScreen(); //重绘
	GetDlgItem(IDC_StopButton)->SetWindowText("暂停");
	GetDlgItem(IDC_StopButton)->EnableWindow(TRUE);
/************************************/
/*数据处理*/
/************************************/	
	FreeRoute(Head); //释放链表
	speed=15;adjust=FALSE;end=FALSE;oncrowd=TRUE; //标记清空
	for(int i=1;i<=71;i++) //清空道路和结点占用
	{
		if(node[i].choose == HIDEFLAG)
			node[i].choose=CLIENTFLAG;
		node[i].father=UNKNOWN;
		node[i].running=FALSE;
		node[i].been=FALSE;
	}
	for(i=1;i<=75;i++)
	{
		road[i].running=FALSE;
		if(road[i].then == TRUE && road[i].traffic == CROWD)
		{	
			road[i].traffic=SMOOTH;
			road[i].then=FALSE;
		}
	}

	//重新赋予其链表头指针
	int first,second,*order=Array(); //此时数组里存的是用户选择的客户
	for(int num=4;*(order+num)!=0;num++); //找数组长度
	Head=Cool(Product(order,num),++num); //退火算法找巡回最短
	free(order);order=NULL; //释放数组内存

	Link=Head->next;first=Link->id;
	xi=node[Link->id].x;
	yi=node[Link->id].y;

	Link=Link->next;second=Link->id;
	sonx=node[Link->id].x;
	sony=node[Link->id].y;

	//文本显示
	AddRunning(first,second);
	V=CalculateSpeed(first,second);
	Statistic();

	timer=TRUE;
	SetTimer(1,speed,NULL);
}
/***************************************************************************************************************************************/
void CMyDlg::OnResetButton() //重置按钮
{
	//中断计时器
	if(timer == TRUE)
	{ 
		timer=FALSE;
		KillTimer(1);
	}
/************************************/
/*数据处理*/
/************************************/
	if(plan != -1 && ing == TRUE)
		FreeRoute(Head); //链表清空
	speed=15;plan=-1;adjust=FALSE;ing=FALSE;end=FALSE;test=FALSE;only=0;fok=TRUE;oncrowd=TRUE; //标记清空

	//删除记录文件
	remove("mindistance.txt");
	remove("CarA.txt");

	for(int i=1;i<=71;i++) //清空残留数据
	{
		node[i].choose=UNKNOWN;
		node[i].running=FALSE;
		node[i].father=UNKNOWN;
		node[i].been=FALSE;
	}
	for(i=1;i<=75;i++)
	{
		road[i].traffic=SMOOTH;
		road[i].running=FALSE;
		road[i].then=FALSE;
	}
/************************************/
/*图像处理*/
/************************************/
	MessageBox("数据已清空，您现在可以重新标注地址。","提示",MB_OK|MB_ICONINFORMATION);

	//状态区清空
	m_NeedEdit="";
	m_FuelEdit="";
	m_OtherEdit="";
	m_MinRadio=-1;
	m_DistanceStr="";
	m_PrimeStr="";
	m_WholeStr="";
	m_SpeedStr="";
	m_TrafficStr="";
	UpdateData(FALSE); //刷新屏幕
	//地图重绘
	Map();
	//模式文字
	if(mode == 0)	GetDlgItem(IDC_ModeStr)->SetWindowText("收敛重找模式");
	else			GetDlgItem(IDC_ModeStr)->SetWindowText("非收敛重找模式");
	//处理控件状态
	GetDlgItem(IDC_NeedEdit)->EnableWindow(TRUE);
	GetDlgItem(IDC_FuelEdit)->EnableWindow(TRUE);
	GetDlgItem(IDC_OtherEdit)->EnableWindow(TRUE);
	GetDlgItem(IDC_MindisRadio)->EnableWindow(TRUE);
	GetDlgItem(IDC_MintimRadio)->EnableWindow(TRUE);

	GetDlgItem(IDC_OkButton)->EnableWindow(TRUE);
	GetDlgItem(IDC_StopButton)->SetWindowText("暂停");
	GetDlgItem(IDC_StopButton)->EnableWindow(FALSE);
	GetDlgItem(IDC_ReplayButton)->EnableWindow(FALSE);
	GetDlgItem(IDC_TestButton)->SetWindowText("测试");
}
/***************************************************************************************************************************************/
void CMyDlg::OnTimer(UINT nIDEvent) //定时器响应动作
{
	static int first,second;
	
	//首先判断之前的横纵坐标是否已经到达下一结点
	if(xi == node[Link->id].x && yi == node[Link->id].y) //若到达新结点
	{
		first=Link->id; //轮换前、次结点

		//到达的是客户
		if(node[Link->id].choose == CLIENTFLAG)
		{
			//优化处理
			PlaySound("Been.wav",NULL,SND_FILENAME|SND_ASYNC); //到达音效
			if(node[Link->id].been == FALSE) PutScreen(); //擦除之前走的红线，预示已经配送到一个客户
			node[Link->id].been=TRUE; //标记为已配送标志
			ShowBeen(); //筛选已配送的客户并将其显示勾

			//清空所有占用
			for(int i=1;i<=71;i++)
				node[i].running=FALSE;
			for(i=1;i<=75;i++)
			{
				road[i].running=FALSE;
				TrafficFlag(i); //重绘拥挤道路
			}
		}

		//下一个结点的取否
		if(Link->next != NULL)//未到表尾
		{
			Link=Link->next; //取下一个结点
			second=Link->id; //次结点改变

			sonx=node[second].x; //取下一结点坐标
			sony=node[second].y;

			//优化处理
			node[second].running=TRUE; //新结点占用
			AddRunning(first,second); //道路占用添加
			ChangeAnytime(first,second); //随时更新速度和路况
		}
		else //到达终点
		{
			end=TRUE; //标志结束
			timer=FALSE; //标志定时器关闭
			KillTimer(1); //删除定时器

			if(test != TRUE)	MessageBox("已完成本次寻路。","提示",MB_OK|MB_ICONINFORMATION);
			GetDlgItem(IDC_StopButton)->EnableWindow(FALSE); //暂停按钮不可选
		}
	}

	//画行进路线
	DrawLine();

	CDialog::OnTimer(nIDEvent);
}
/***************************************************************************************************************************************/
void CMyDlg::OnRButtonDown(UINT nFlags, CPoint point) //右键消息
{	
	//点击地图区
	if((point.x>=180&&point.x<=780)&&(point.y>=11&&point.y<=611))
	{
		//初始化菜单有效
		CMenu *pMenu=m_Menu.GetSubMenu(0);
		if(only == 0)	pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_ENABLED);
		else       	    pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);
		pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_ENABLED);
		pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);

		//点击地址和道路时的旗子位置处理
		Point1.x=point.x;Point1.y=point.y; //把用户点击坐标传给全局坐标（以备在其他响应函数中用）
		if	   ((point.x>=227&&point.x<=237)&&(point.y>=226&&point.y<=236))	{Point2.x=232;Point2.y=231;if(node[51].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=227&&point.x<=237)&&(point.y>=297&&point.y<=307))	{Point2.x=232;Point2.y=302;if(node[52].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=227&&point.x<=237)&&(point.y>=377&&point.y<=387)) {Point2.x=232;Point2.y=382;if(node[53].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=233&&point.x<=243)&&(point.y>=458&&point.y<=468)) {Point2.x=238;Point2.y=463;if(node[54].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=315&&point.x<=325)&&(point.y>=223&&point.y<=233)) {Point2.x=320;Point2.y=228;if(node[55].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=317&&point.x<=327)&&(point.y>=460&&point.y<=470)) {Point2.x=320;Point2.y=465;if(node[56].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=450&&point.x<=460)&&(point.y>=373&&point.y<=383)) {Point2.x=455;Point2.y=378;if(node[57].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=510&&point.x<=520)&&(point.y>=35 &&point.y<=45 )) {Point2.x=515;Point2.y=40 ;if(node[58].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=510&&point.x<=520)&&(point.y>=99 &&point.y<=109)) {Point2.x=515;Point2.y=104;if(node[59].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=513&&point.x<=523)&&(point.y>=203&&point.y<=213)) {Point2.x=518;Point2.y=208;if(node[60].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=510&&point.x<=520)&&(point.y>=290&&point.y<=300)) {Point2.x=515;Point2.y=295;if(node[61].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}		
		else if((point.x>=480&&point.x<=490)&&(point.y>=454&&point.y<=464)) {Point2.x=485;Point2.y=459;if(node[62].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=541&&point.x<=551)&&(point.y>=160&&point.y<=170)) {Point2.x=546;Point2.y=165;if(node[63].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=548&&point.x<=558)&&(point.y>=359&&point.y<=369)) {Point2.x=553;Point2.y=364;if(node[64].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=541&&point.x<=551)&&(point.y>=390&&point.y<=400)) {Point2.x=546;Point2.y=395;if(node[65].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=543&&point.x<=553)&&(point.y>=432&&point.y<=442)) {Point2.x=558;Point2.y=447;if(node[66].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=541&&point.x<=551)&&(point.y>=470&&point.y<=480)) {Point2.x=546;Point2.y=475;if(node[67].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=510&&point.x<=520)&&(point.y>=591&&point.y<=601)) {Point2.x=515;Point2.y=596;if(node[68].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=660&&point.x<=670)&&(point.y>=412&&point.y<=422)) {Point2.x=680;Point2.y=432;if(node[69].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=625&&point.x<=635)&&(point.y>=510&&point.y<=520)) {Point2.x=607;Point2.y=538;if(node[70].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}
		else if((point.x>=728&&point.x<=738)&&(point.y>=448&&point.y<=458)) {Point2.x=733;Point2.y=453;if(node[71].choose!=UNKNOWN||ing==TRUE||end==TRUE) {pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);}}	
		else
		{
			//仓库、客户标记不可选、路况可选
			pMenu->EnableMenuItem(0,MF_BYPOSITION|MF_GRAYED);
			pMenu->EnableMenuItem(1,MF_BYPOSITION|MF_GRAYED);
			pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_ENABLED);

			//决定路况可不可选
			if     ((Point1.x>=219&&Point1.x<=225)&&(Point1.y>=231&&Point1.y<=271)) {if(road[1 ].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=219&&Point1.x<=225)&&(Point1.y>=271&&Point1.y<=333)) {if(road[2 ].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=219&&Point1.x<=224)&&(Point1.y>=333&&Point1.y<=416)) {if(road[3 ].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=219&&Point1.x<=225)&&(Point1.y>=416&&Point1.y<=494)) {if(road[4 ].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=224&&Point1.x<=306)&&(Point1.y>=268&&Point1.y<=274)) {if(road[5 ].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=306&&Point1.x<=338)&&(Point1.y>=268&&Point1.y<=283)) {if(road[6 ].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=222&&Point1.x<=338)&&(Point1.y>=330&&Point1.y<=336)) {if(road[7 ].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=222&&Point1.x<=338)&&(Point1.y>=413&&Point1.y<=419)) {if(road[8 ].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=222&&Point1.x<=340)&&(Point1.y>=491&&Point1.y<=497)) {if(road[9 ].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=337&&Point1.x<=343)&&(Point1.y>=228&&Point1.y<=280)) {if(road[10].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=337&&Point1.x<=343)&&(Point1.y>=277&&Point1.y<=333)) {if(road[11].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=337&&Point1.x<=342)&&(Point1.y>=333&&Point1.y<=414)) {if(road[12].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=337&&Point1.x<=343)&&(Point1.y>=414&&Point1.y<=494)) {if(road[13].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=340&&Point1.x<=384)&&(Point1.y>=277&&Point1.y<=282)) {if(road[14].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=340&&Point1.x<=384)&&(Point1.y>=332&&Point1.y<=336)) {if(road[15].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=338&&Point1.x<=426)&&(Point1.y>=413&&Point1.y<=418)) {if(road[16].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=340&&Point1.x<=426)&&(Point1.y>=491&&Point1.y<=497)) {if(road[17].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=381&&Point1.x<=387)&&(Point1.y>=280&&Point1.y<=333)) {if(road[18].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=423&&Point1.x<=429)&&(Point1.y>=332&&Point1.y<=413)) {if(road[19].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=423&&Point1.x<=429)&&(Point1.y>=413&&Point1.y<=493)) {if(road[20].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=384&&Point1.x<=476)&&(Point1.y>=277&&Point1.y<=281)) {if(road[21].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=384&&Point1.x<=426)&&(Point1.y>=330&&Point1.y<=336)) {if(road[22].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=426&&Point1.x<=476)&&(Point1.y>=330&&Point1.y<=335)) {if(road[23].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=426&&Point1.x<=476)&&(Point1.y>=412&&Point1.y<=416)) {if(road[24].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=426&&Point1.x<=476)&&(Point1.y>=490&&Point1.y<=496)) {if(road[25].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=473&&Point1.x<=477)&&(Point1.y>=218&&Point1.y<=277)) {if(road[26].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=473&&Point1.x<=477)&&(Point1.y>=277&&Point1.y<=330)) {if(road[27].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=473&&Point1.x<=477)&&(Point1.y>=330&&Point1.y<=413)) {if(road[28].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=473&&Point1.x<=477)&&(Point1.y>=413&&Point1.y<=492)) {if(road[29].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=476&&Point1.x<=506)&&(Point1.y>=215&&Point1.y<=222)) {if(road[30].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=477&&Point1.x<=493)&&(Point1.y>=274&&Point1.y<=288)) {if(road[31].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=493&&Point1.x<=503)&&(Point1.y>=265&&Point1.y<=285)) {if(road[32].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=477&&Point1.x<=493)&&(Point1.y>=285&&Point1.y<=328)) {if(road[33].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
		    else if((Point1.x>=476&&Point1.x<=505)&&(Point1.y>=327&&Point1.y<=333)) {if(road[34].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=476&&Point1.x<=505)&&(Point1.y>=410&&Point1.y<=416)) {if(road[35].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=476&&Point1.x<=538)&&(Point1.y>=490&&Point1.y<=495)) {if(road[36].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=502&&Point1.x<=508)&&(Point1.y>=40 &&Point1.y<=68 )) {if(road[37].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=502&&Point1.x<=508)&&(Point1.y>=68 &&Point1.y<=123)) {if(road[38].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=504&&Point1.x<=508)&&(Point1.y>=123&&Point1.y<=218)) {if(road[39].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=503&&Point1.x<=508)&&(Point1.y>=218&&Point1.y<=265)) {if(road[40].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=503&&Point1.x<=508)&&(Point1.y>=265&&Point1.y<=330)) {if(road[41].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=503&&Point1.x<=508)&&(Point1.y>=330&&Point1.y<=413)) {if(road[42].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=505&&Point1.x<=538)&&(Point1.y>=65 &&Point1.y<=71 )) {if(road[43].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=505&&Point1.x<=538)&&(Point1.y>=120&&Point1.y<=126)) {if(road[44].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=505&&Point1.x<=529)&&(Point1.y>=215&&Point1.y<=221)) {if(road[45].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=505&&Point1.x<=538)&&(Point1.y>=327&&Point1.y<=333)) {if(road[46].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=505&&Point1.x<=538)&&(Point1.y>=410&&Point1.y<=416)) {if(road[47].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=535&&Point1.x<=539)&&(Point1.y>=68 &&Point1.y<=123)) {if(road[48].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=535&&Point1.x<=541)&&(Point1.y>=123&&Point1.y<=218)) {if(road[49].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=535&&Point1.x<=539)&&(Point1.y>=218&&Point1.y<=286)) {if(road[50].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=535&&Point1.x<=539)&&(Point1.y>=286&&Point1.y<=330)) {if(road[51].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}		
			else if((Point1.x>=535&&Point1.x<=540)&&(Point1.y>=330&&Point1.y<=413)) {if(road[52].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=535&&Point1.x<=541)&&(Point1.y>=413&&Point1.y<=491)) {if(road[53].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=535&&Point1.x<=541)&&(Point1.y>=491&&Point1.y<=596)) {if(road[54].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=538&&Point1.x<=612)&&(Point1.y>=327&&Point1.y<=331)) {if(road[55].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=538&&Point1.x<=584)&&(Point1.y>=410&&Point1.y<=416)) {if(road[56].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=584&&Point1.x<=648)&&(Point1.y>=412&&Point1.y<=424)) {if(road[57].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=541&&Point1.x<=595)&&(Point1.y>=491&&Point1.y<=506)) {if(road[58].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=595&&Point1.x<=642)&&(Point1.y>=503&&Point1.y<=511)) {if(road[59].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=610&&Point1.x<=631)&&(Point1.y>=330&&Point1.y<=363)) {if(road[60].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=631&&Point1.x<=651)&&(Point1.y>=363&&Point1.y<=386)) {if(road[61].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=647&&Point1.x<=653)&&(Point1.y>=385&&Point1.y<=424)) {if(road[62].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=646&&Point1.x<=651)&&(Point1.y>=424&&Point1.y<=456)) {if(road[63].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=645&&Point1.x<=648)&&(Point1.y>=456&&Point1.y<=469)) {if(road[64].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=640&&Point1.x<=647)&&(Point1.y>=469&&Point1.y<=508)) {if(road[65].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=653&&Point1.x<=710)&&(Point1.y>=385&&Point1.y<=408)) {if(road[66].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=650&&Point1.x<=665)&&(Point1.y>=422&&Point1.y<=427)) {if(road[67].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=642&&Point1.x<=678)&&(Point1.y>=506&&Point1.y<=515)) {if(road[68].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=678&&Point1.x<=701)&&(Point1.y>=509&&Point1.y<=515)) {if(road[69].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=701&&Point1.x<=712)&&(Point1.y>=515&&Point1.y<=529)) {if(road[70].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=710&&Point1.x<=723)&&(Point1.y>=408&&Point1.y<=432)) {if(road[71].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=720&&Point1.x<=725)&&(Point1.y>=433&&Point1.y<=478)) {if(road[72].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=711&&Point1.x<=722)&&(Point1.y>=478&&Point1.y<=530)) {if(road[73].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=529&&Point1.x<=538)&&(Point1.y>=215&&Point1.y<=221)) {if(road[74].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else if((Point1.x>=508&&Point1.x<=535)&&(Point1.y>=223&&Point1.y<=265)) {if(road[75].running==TRUE||end==TRUE||test==TRUE) {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);}}
			else   {pMenu->EnableMenuItem(2,MF_BYPOSITION|MF_GRAYED);} //多余区
		}
		//点击处显示弹出式菜单
		ClientToScreen(&point);
		CRect rect;
		rect.top=point.x;
		rect.left=point.y;
		pMenu->TrackPopupMenu(TPM_LEFTALIGN | TPM_LEFTBUTTON | TPM_VERTICAL,rect.top,rect.left,this,&rect);
	}

	CDialog::OnRButtonDown(nFlags, point);
}
/***************************************************************************************************************************************/
void CMyDlg::OnMouseMove(UINT nFlags, CPoint point) //鼠标移动
{
	//鼠标坐标显示
	if((point.x>=179&&point.x<=779)&&(point.y>=11&&point.y<=611))
	{
		CString str;
		str.Format("%d",point.x);
		GetDlgItem(IDC_x)->SetWindowText(str);
		str.Format("%d",point.y);
		GetDlgItem(IDC_y)->SetWindowText(str);
	}
	else
	{
		CString str="";
		GetDlgItem(IDC_x)->SetWindowText(str);
		GetDlgItem(IDC_y)->SetWindowText(str);
	}
	//地址选中效果
	if		((point.x>=227&&point.x<=237)&&(point.y>=226&&point.y<=236)){LightBlue(232,231); touch=ONBLUE;}
	else if((point.x>=227&&point.x<=237)&&(point.y>=297&&point.y<=307))	{LightBlue(232,302); touch=ONBLUE;}
	else if((point.x>=227&&point.x<=237)&&(point.y>=377&&point.y<=387)) {LightBlue(232,382); touch=ONBLUE;}
	else if((point.x>=233&&point.x<=243)&&(point.y>=458&&point.y<=468)) {LightBlue(238,463); touch=ONBLUE;}
	else if((point.x>=315&&point.x<=325)&&(point.y>=223&&point.y<=233)) {LightBlue(320,228); touch=ONBLUE;}
	else if((point.x>=317&&point.x<=327)&&(point.y>=460&&point.y<=470)) {LightBlue(322,465); touch=ONBLUE;}
	else if((point.x>=510&&point.x<=520)&&(point.y>=35 &&point.y<=45 )) {LightBlue(515,40 ); touch=ONBLUE;}
	else if((point.x>=510&&point.x<=520)&&(point.y>=99 &&point.y<=109)) {LightBlue(515,104); touch=ONBLUE;}
	else if((point.x>=541&&point.x<=551)&&(point.y>=160&&point.y<=170)) {LightBlue(546,165); touch=ONBLUE;}
	else if((point.x>=513&&point.x<=523)&&(point.y>=203&&point.y<=213)) {LightBlue(518,208); touch=ONBLUE;}
	else if((point.x>=510&&point.x<=520)&&(point.y>=290&&point.y<=300)) {LightBlue(515,295); touch=ONBLUE;}
	else if((point.x>=450&&point.x<=460)&&(point.y>=373&&point.y<=383)) {LightBlue(455,378); touch=ONBLUE;}
	else if((point.x>=480&&point.x<=490)&&(point.y>=454&&point.y<=464)) {LightBlue(485,459); touch=ONBLUE;}
	else if((point.x>=548&&point.x<=558)&&(point.y>=359&&point.y<=369)) {LightBlue(553,364); touch=ONBLUE;}
	else if((point.x>=541&&point.x<=551)&&(point.y>=390&&point.y<=400)) {LightBlue(546,395); touch=ONBLUE;}
	else if((point.x>=543&&point.x<=553)&&(point.y>=432&&point.y<=442)) {LightBlue(548,437); touch=ONBLUE;}
	else if((point.x>=541&&point.x<=551)&&(point.y>=470&&point.y<=480)) {LightBlue(546,475); touch=ONBLUE;}
	else if((point.x>=510&&point.x<=520)&&(point.y>=591&&point.y<=601)) {LightBlue(515,596); touch=ONBLUE;}
	else if((point.x>=625&&point.x<=635)&&(point.y>=510&&point.y<=520)) {LightBlue(630,515); touch=ONBLUE;}
	else if((point.x>=660&&point.x<=670)&&(point.y>=412&&point.y<=422)) {LightBlue(665,417); touch=ONBLUE;}
	else if((point.x>=728&&point.x<=738)&&(point.y>=448&&point.y<=458)) {LightBlue(733,453); touch=ONBLUE;}
	
	//道路选中效果
	else if((point.x>=219&&point.x<=225)&&(point.y>=231&&point.y<=271)&&road[1 ].traffic==SMOOTH&&road[1 ].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(219,231,225,271); touch=1; }
	else if((point.x>=219&&point.x<=225)&&(point.y>=271&&point.y<=333)&&road[2 ].traffic==SMOOTH&&road[2 ].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(219,271,225,333); touch=2; }
	else if((point.x>=219&&point.x<=224)&&(point.y>=333&&point.y<=416)&&road[3 ].traffic==SMOOTH&&road[3 ].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(219,333,224,416); touch=3; }
	else if((point.x>=219&&point.x<=225)&&(point.y>=416&&point.y<=494)&&road[4 ].traffic==SMOOTH&&road[4 ].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(219,416,225,494); touch=4; }
	else if((point.x>=224&&point.x<=306)&&(point.y>=268&&point.y<=274)&&road[5 ].traffic==SMOOTH&&road[5 ].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(224,268,306,274); touch=5; }
	else if((point.x>=306&&point.x<=338)&&(point.y>=268&&point.y<=283)&&road[6 ].traffic==SMOOTH&&road[6 ].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(306,268,338,283); touch=6; }
	else if((point.x>=222&&point.x<=338)&&(point.y>=330&&point.y<=336)&&road[7 ].traffic==SMOOTH&&road[7 ].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(222,330,338,336); touch=7; }
	else if((point.x>=222&&point.x<=338)&&(point.y>=413&&point.y<=419)&&road[8 ].traffic==SMOOTH&&road[8 ].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(222,413,338,419); touch=8; }
	else if((point.x>=222&&point.x<=340)&&(point.y>=491&&point.y<=497)&&road[9 ].traffic==SMOOTH&&road[9 ].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(222,491,340,497); touch=9; }
	else if((point.x>=337&&point.x<=343)&&(point.y>=228&&point.y<=280)&&road[10].traffic==SMOOTH&&road[10].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(337,228,343,280); touch=10;}
	else if((point.x>=337&&point.x<=343)&&(point.y>=277&&point.y<=333)&&road[11].traffic==SMOOTH&&road[11].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(337,277,343,333); touch=11;}
	else if((point.x>=337&&point.x<=342)&&(point.y>=333&&point.y<=414)&&road[12].traffic==SMOOTH&&road[12].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(337,333,342,414); touch=12;}
	else if((point.x>=337&&point.x<=343)&&(point.y>=414&&point.y<=494)&&road[13].traffic==SMOOTH&&road[13].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(337,414,343,494); touch=13;}
	else if((point.x>=340&&point.x<=384)&&(point.y>=277&&point.y<=282)&&road[14].traffic==SMOOTH&&road[14].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(340,277,384,282); touch=14;}
	else if((point.x>=340&&point.x<=384)&&(point.y>=332&&point.y<=336)&&road[15].traffic==SMOOTH&&road[15].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(340,332,384,336); touch=15;}
	else if((point.x>=338&&point.x<=426)&&(point.y>=413&&point.y<=418)&&road[16].traffic==SMOOTH&&road[16].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(338,413,426,418); touch=16;}
	else if((point.x>=340&&point.x<=426)&&(point.y>=491&&point.y<=497)&&road[17].traffic==SMOOTH&&road[17].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(340,491,426,497); touch=17;}
	else if((point.x>=381&&point.x<=387)&&(point.y>=280&&point.y<=333)&&road[18].traffic==SMOOTH&&road[18].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(381,280,387,333); touch=18;}
	else if((point.x>=423&&point.x<=429)&&(point.y>=332&&point.y<=413)&&road[19].traffic==SMOOTH&&road[19].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(423,332,429,413); touch=19;}
	else if((point.x>=423&&point.x<=429)&&(point.y>=413&&point.y<=493)&&road[20].traffic==SMOOTH&&road[20].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(423,413,429,493); touch=20;}
	else if((point.x>=384&&point.x<=476)&&(point.y>=277&&point.y<=281)&&road[21].traffic==SMOOTH&&road[21].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(384,277,476,281); touch=21;}
	else if((point.x>=384&&point.x<=426)&&(point.y>=330&&point.y<=336)&&road[22].traffic==SMOOTH&&road[22].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(384,330,426,336); touch=22;}
	else if((point.x>=426&&point.x<=476)&&(point.y>=330&&point.y<=335)&&road[23].traffic==SMOOTH&&road[23].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(426,330,476,335); touch=23;}
	else if((point.x>=426&&point.x<=476)&&(point.y>=412&&point.y<=416)&&road[24].traffic==SMOOTH&&road[24].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(426,412,476,416); touch=24;}
	else if((point.x>=426&&point.x<=476)&&(point.y>=490&&point.y<=496)&&road[25].traffic==SMOOTH&&road[25].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(426,490,476,496); touch=25;}
	else if((point.x>=473&&point.x<=477)&&(point.y>=218&&point.y<=277)&&road[26].traffic==SMOOTH&&road[26].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(473,218,477,277); touch=26;}
	else if((point.x>=473&&point.x<=477)&&(point.y>=277&&point.y<=330)&&road[27].traffic==SMOOTH&&road[27].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(473,277,477,330); touch=27;}
	else if((point.x>=473&&point.x<=477)&&(point.y>=330&&point.y<=413)&&road[28].traffic==SMOOTH&&road[28].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(473,330,477,413); touch=28;}
	else if((point.x>=473&&point.x<=477)&&(point.y>=413&&point.y<=492)&&road[29].traffic==SMOOTH&&road[29].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(473,413,477,492); touch=29;}
	else if((point.x>=476&&point.x<=506)&&(point.y>=215&&point.y<=222)&&road[30].traffic==SMOOTH&&road[30].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(476,215,506,222); touch=30;}
	else if((point.x>=477&&point.x<=493)&&(point.y>=274&&point.y<=288)&&road[31].traffic==SMOOTH&&road[31].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(477,274,493,288); touch=31;}
	else if((point.x>=493&&point.x<=503)&&(point.y>=265&&point.y<=285)&&road[32].traffic==SMOOTH&&road[32].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(493,265,503,285); touch=32;}
    else if((point.x>=477&&point.x<=493)&&(point.y>=285&&point.y<=328)&&road[33].traffic==SMOOTH&&road[33].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(477,285,493,328); touch=33;}
    else if((point.x>=476&&point.x<=505)&&(point.y>=327&&point.y<=333)&&road[34].traffic==SMOOTH&&road[34].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(476,327,505,333); touch=34;}
	else if((point.x>=476&&point.x<=505)&&(point.y>=410&&point.y<=416)&&road[35].traffic==SMOOTH&&road[35].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(476,410,505,416); touch=35;}
	else if((point.x>=476&&point.x<=538)&&(point.y>=490&&point.y<=495)&&road[36].traffic==SMOOTH&&road[36].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(476,490,538,495); touch=36;}
	else if((point.x>=502&&point.x<=508)&&(point.y>=40 &&point.y<=68 )&&road[37].traffic==SMOOTH&&road[37].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(502,40,508,68  ); touch=37;}
	else if((point.x>=502&&point.x<=508)&&(point.y>=68 &&point.y<=123)&&road[38].traffic==SMOOTH&&road[38].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(502,68,508,123 ); touch=38;}
	else if((point.x>=504&&point.x<=508)&&(point.y>=123&&point.y<=218)&&road[39].traffic==SMOOTH&&road[39].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(504,123,508,218); touch=39;}
	else if((point.x>=503&&point.x<=508)&&(point.y>=218&&point.y<=265)&&road[40].traffic==SMOOTH&&road[40].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(503,218,508,265); touch=40;}
	else if((point.x>=503&&point.x<=508)&&(point.y>=265&&point.y<=330)&&road[41].traffic==SMOOTH&&road[41].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(503,265,508,330); touch=41;}
	else if((point.x>=503&&point.x<=508)&&(point.y>=330&&point.y<=413)&&road[42].traffic==SMOOTH&&road[42].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(503,330,508,413); touch=42;}
	else if((point.x>=505&&point.x<=538)&&(point.y>=65 &&point.y<=71 )&&road[43].traffic==SMOOTH&&road[43].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(505,65,538,71  ); touch=43;}
	else if((point.x>=505&&point.x<=538)&&(point.y>=120&&point.y<=126)&&road[44].traffic==SMOOTH&&road[44].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(505,120,538,126); touch=44;}
	else if((point.x>=505&&point.x<=529)&&(point.y>=215&&point.y<=221)&&road[45].traffic==SMOOTH&&road[45].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(505,215,529,221); touch=45;}
	else if((point.x>=505&&point.x<=538)&&(point.y>=327&&point.y<=333)&&road[46].traffic==SMOOTH&&road[46].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(505,327,538,333); touch=46;}
	else if((point.x>=505&&point.x<=538)&&(point.y>=410&&point.y<=416)&&road[47].traffic==SMOOTH&&road[47].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(505,410,538,416); touch=47;}
	else if((point.x>=535&&point.x<=539)&&(point.y>=68 &&point.y<=123)&&road[48].traffic==SMOOTH&&road[48].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(535,68,539,123 ); touch=48;}
	else if((point.x>=535&&point.x<=541)&&(point.y>=123&&point.y<=218)&&road[49].traffic==SMOOTH&&road[49].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(535,123,541,218); touch=49;}
	else if((point.x>=535&&point.x<=539)&&(point.y>=218&&point.y<=286)&&road[50].traffic==SMOOTH&&road[50].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(535,218,539,286); touch=50;}
	else if((point.x>=535&&point.x<=539)&&(point.y>=286&&point.y<=330)&&road[51].traffic==SMOOTH&&road[51].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(535,286,539,330); touch=51;}
	else if((point.x>=535&&point.x<=540)&&(point.y>=330&&point.y<=413)&&road[52].traffic==SMOOTH&&road[52].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(535,330,540,413); touch=52;}
	else if((point.x>=535&&point.x<=541)&&(point.y>=413&&point.y<=491)&&road[53].traffic==SMOOTH&&road[53].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(535,413,541,491); touch=53;}
	else if((point.x>=535&&point.x<=541)&&(point.y>=491&&point.y<=596)&&road[54].traffic==SMOOTH&&road[54].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(535,491,541,596); touch=54;}
	else if((point.x>=538&&point.x<=612)&&(point.y>=327&&point.y<=331)&&road[55].traffic==SMOOTH&&road[55].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(538,327,612,331); touch=55;}
	else if((point.x>=538&&point.x<=584)&&(point.y>=410&&point.y<=416)&&road[56].traffic==SMOOTH&&road[56].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(538,410,584,416); touch=56;}
	else if((point.x>=584&&point.x<=648)&&(point.y>=412&&point.y<=424)&&road[57].traffic==SMOOTH&&road[57].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(584,412,648,424); touch=57;}
	else if((point.x>=541&&point.x<=595)&&(point.y>=491&&point.y<=506)&&road[58].traffic==SMOOTH&&road[58].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(541,491,595,506); touch=58;}
	else if((point.x>=595&&point.x<=642)&&(point.y>=503&&point.y<=511)&&road[59].traffic==SMOOTH&&road[59].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(595,503,642,511); touch=59;}
	else if((point.x>=610&&point.x<=631)&&(point.y>=330&&point.y<=363)&&road[60].traffic==SMOOTH&&road[60].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(610,330,631,363); touch=60;}
	else if((point.x>=631&&point.x<=651)&&(point.y>=363&&point.y<=386)&&road[61].traffic==SMOOTH&&road[61].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(631,363,651,386); touch=61;}
	else if((point.x>=647&&point.x<=653)&&(point.y>=385&&point.y<=424)&&road[62].traffic==SMOOTH&&road[62].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(647,385,653,424); touch=62;}
	else if((point.x>=646&&point.x<=651)&&(point.y>=424&&point.y<=456)&&road[63].traffic==SMOOTH&&road[63].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(646,424,651,456); touch=63;}
	else if((point.x>=645&&point.x<=648)&&(point.y>=456&&point.y<=469)&&road[64].traffic==SMOOTH&&road[64].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(645,456,651,469); touch=64;}
	else if((point.x>=640&&point.x<=647)&&(point.y>=469&&point.y<=508)&&road[65].traffic==SMOOTH&&road[65].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(640,469,647,508); touch=65;}
	else if((point.x>=653&&point.x<=710)&&(point.y>=385&&point.y<=408)&&road[66].traffic==SMOOTH&&road[66].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(653,385,710,408); touch=66;}
	else if((point.x>=650&&point.x<=665)&&(point.y>=422&&point.y<=427)&&road[67].traffic==SMOOTH&&road[67].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(650,422,665,427); touch=67;}
	else if((point.x>=642&&point.x<=678)&&(point.y>=506&&point.y<=515)&&road[68].traffic==SMOOTH&&road[68].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(642,506,678,515); touch=68;}
	else if((point.x>=678&&point.x<=701)&&(point.y>=509&&point.y<=515)&&road[69].traffic==SMOOTH&&road[69].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(678,509,701,515); touch=69;}
	else if((point.x>=701&&point.x<=712)&&(point.y>=515&&point.y<=529)&&road[70].traffic==SMOOTH&&road[70].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(698,515,712,529); touch=70;}
	else if((point.x>=710&&point.x<=723)&&(point.y>=408&&point.y<=432)&&road[71].traffic==SMOOTH&&road[71].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(710,408,723,432); touch=71;}
	else if((point.x>=720&&point.x<=725)&&(point.y>=433&&point.y<=478)&&road[72].traffic==SMOOTH&&road[72].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(720,433,725,478); touch=72;}
	else if((point.x>=711&&point.x<=722)&&(point.y>=480&&point.y<=530)&&road[73].traffic==SMOOTH&&road[73].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(711,478,722,530); touch=73;}
	else if((point.x>=529&&point.x<=538)&&(point.y>=215&&point.y<=221)&&road[74].traffic==SMOOTH&&road[74].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(529,215,538,221); touch=74;}
	else if((point.x>=508&&point.x<=535)&&(point.y>=223&&point.y<=265)&&road[75].traffic==SMOOTH&&road[75].running==FALSE&&touch==OUTDOOR&&test!=TRUE) {OnPicture(508,223,535,265); touch=75;}

	else //鼠标在空白处
	{
		if(touch == ONBLUE) //擦除选中地址
		{
			AllBlue();
			touch=OUTDOOR;
		}
		if(touch != OUTDOOR&&touch != ONBLUE) //擦除选中道路
		{
			switch(touch)
			{
				case 1 :Picture(219,231,225,271);touch=OUTDOOR;break;
				case 2 :Picture(219,271,225,333);touch=OUTDOOR;break;
				case 3 :Picture(219,333,224,416);touch=OUTDOOR;break;
				case 4 :Picture(219,416,225,494);touch=OUTDOOR;break;
				case 5 :Picture(224,268,306,274);touch=OUTDOOR;break;
				case 6 :Picture(306,268,338,283);touch=OUTDOOR;break;
				case 7 :Picture(222,330,338,336);touch=OUTDOOR;break;
				case 8 :Picture(222,413,338,419);touch=OUTDOOR;break;
				case 9 :Picture(222,491,340,497);touch=OUTDOOR;break;
				case 10:Picture(337,228,343,280);touch=OUTDOOR;break;
				case 11:Picture(337,277,343,333);touch=OUTDOOR;break;
				case 12:Picture(337,333,342,414);touch=OUTDOOR;break;
				case 13:Picture(337,414,343,494);touch=OUTDOOR;break;
				case 14:Picture(340,277,384,282);touch=OUTDOOR;break;
				case 15:Picture(340,332,384,336);touch=OUTDOOR;break;
				case 16:Picture(338,413,426,418);touch=OUTDOOR;break;
				case 17:Picture(340,491,426,497);touch=OUTDOOR;break;
				case 18:Picture(381,280,387,333);touch=OUTDOOR;break;
				case 19:Picture(423,332,429,413);touch=OUTDOOR;break;
				case 20:Picture(423,413,429,493);touch=OUTDOOR;break;
				case 21:Picture(384,277,476,281);touch=OUTDOOR;break;
				case 22:Picture(384,330,426,336);touch=OUTDOOR;break;
				case 23:Picture(426,330,476,335);touch=OUTDOOR;break;
				case 24:Picture(426,412,476,416);touch=OUTDOOR;break;
				case 25:Picture(426,490,476,496);touch=OUTDOOR;break;
				case 26:Picture(473,218,477,277);touch=OUTDOOR;break;
				case 27:Picture(473,277,477,330);touch=OUTDOOR;break;
				case 28:Picture(473,330,477,413);touch=OUTDOOR;break;
				case 29:Picture(473,413,477,492);touch=OUTDOOR;break;
				case 30:Picture(476,215,506,222);touch=OUTDOOR;break;
				case 31:Picture(476,274,493,288);touch=OUTDOOR;break;
				case 32:Picture(493,265,503,285);touch=OUTDOOR;break;
				case 33:Picture(477,285,493,328);touch=OUTDOOR;break;
				case 34:Picture(476,327,505,333);touch=OUTDOOR;break;
				case 35:Picture(476,410,505,416);touch=OUTDOOR;break;
				case 36:Picture(476,490,538,495);touch=OUTDOOR;break;
				case 37:Picture(502,40,508,68  );touch=OUTDOOR;break;
				case 38:Picture(502,68,508,123 );touch=OUTDOOR;break;
				case 39:Picture(504,123,508,218);touch=OUTDOOR;break;
				case 40:Picture(503,218,508,265);touch=OUTDOOR;break;
				case 41:Picture(503,265,508,330);touch=OUTDOOR;break;
				case 42:Picture(503,330,508,413);touch=OUTDOOR;break;
				case 43:Picture(505,65,538,71  );touch=OUTDOOR;break;
				case 44:Picture(505,120,538,126);touch=OUTDOOR;break;
				case 45:Picture(505,215,529,221);touch=OUTDOOR;break;
				case 46:Picture(505,327,538,333);touch=OUTDOOR;break;
				case 47:Picture(505,410,538,416);touch=OUTDOOR;break;
				case 48:Picture(535,68,539,123 );touch=OUTDOOR;break;
				case 49:Picture(535,123,541,218);touch=OUTDOOR;break;
				case 50:Picture(535,218,539,286);touch=OUTDOOR;break;
				case 51:Picture(535,286,539,330);touch=OUTDOOR;break;
				case 52:Picture(535,330,540,413);touch=OUTDOOR;break;
				case 53:Picture(535,413,541,491);touch=OUTDOOR;break;
				case 54:Picture(535,491,541,596);touch=OUTDOOR;break;
				case 55:Picture(538,327,612,331);touch=OUTDOOR;break;
				case 56:Picture(538,410,584,416);touch=OUTDOOR;break;
				case 57:Picture(584,412,648,424);touch=OUTDOOR;break;
				case 58:Picture(541,491,595,506);touch=OUTDOOR;break;
				case 59:Picture(595,503,642,511);touch=OUTDOOR;break;
				case 60:Picture(610,330,631,363);touch=OUTDOOR;break;
				case 61:Picture(631,363,651,386);touch=OUTDOOR;break;
				case 62:Picture(647,385,653,424);touch=OUTDOOR;break;
				case 63:Picture(646,424,651,456);touch=OUTDOOR;break;
				case 64:Picture(645,456,651,469);touch=OUTDOOR;break;
				case 65:Picture(640,469,647,508);touch=OUTDOOR;break;
				case 66:Picture(653,385,710,408);touch=OUTDOOR;break;
				case 67:Picture(650,422,665,427);touch=OUTDOOR;break;
				case 68:Picture(642,506,678,515);touch=OUTDOOR;break;
				case 69:Picture(678,509,701,515);touch=OUTDOOR;break;
				case 70:Picture(698,515,712,529);touch=OUTDOOR;break;
				case 71:Picture(710,408,723,432);touch=OUTDOOR;break;
				case 72:Picture(720,433,725,478);touch=OUTDOOR;break;
				case 73:Picture(711,478,722,530);touch=OUTDOOR;break;
				case 74:Picture(529,215,538,221);touch=OUTDOOR;break;
				case 75:Picture(508,223,535,265);touch=OUTDOOR;break;
			}
			touch=OUTDOOR; //还原标记成没有选中状态
		}
	}

	CDialog::OnMouseMove(nFlags, point);
}
/***************************************************************************************************************************************/
void CMyDlg::OnStoreMenu() //仓库标记菜单
{
	if(only == 0) //未标记仓库
	{
		//显示仓库旗
		StoreFlag();

		//识别仓库地址并标记它的属性
		if	   ((Point1.x>=227&&Point1.x<=237)&&(Point1.y>=226&&Point1.y<=236))	{Beep(1200,500);node[51].choose=STOREFLAG;only=1;}
		else if((Point1.x>=227&&Point1.x<=237)&&(Point1.y>=297&&Point1.y<=307))	{Beep(1200,500);node[52].choose=STOREFLAG;only=1;}
		else if((Point1.x>=227&&Point1.x<=237)&&(Point1.y>=377&&Point1.y<=387)) {Beep(1200,500);node[53].choose=STOREFLAG;only=1;}	
		else if((Point1.x>=233&&Point1.x<=243)&&(Point1.y>=458&&Point1.y<=468)) {Beep(1200,500);node[54].choose=STOREFLAG;only=1;}
		else if((Point1.x>=315&&Point1.x<=325)&&(Point1.y>=223&&Point1.y<=233)) {Beep(1200,500);node[55].choose=STOREFLAG;only=1;}
		else if((Point1.x>=317&&Point1.x<=327)&&(Point1.y>=460&&Point1.y<=470)) {Beep(1200,500);node[56].choose=STOREFLAG;only=1;}
		else if((Point1.x>=450&&Point1.x<=460)&&(Point1.y>=373&&Point1.y<=383)) {Beep(1200,500);node[57].choose=STOREFLAG;only=1;}
		else if((Point1.x>=510&&Point1.x<=520)&&(Point1.y>=35 &&Point1.y<=45 )) {Beep(1200,500);node[58].choose=STOREFLAG;only=1;}
		else if((Point1.x>=510&&Point1.x<=520)&&(Point1.y>=99 &&Point1.y<=109)) {Beep(1200,500);node[59].choose=STOREFLAG;only=1;}
		else if((Point1.x>=513&&Point1.x<=523)&&(Point1.y>=203&&Point1.y<=213)) {Beep(1200,500);node[60].choose=STOREFLAG;only=1;}
		else if((Point1.x>=510&&Point1.x<=520)&&(Point1.y>=290&&Point1.y<=300)) {Beep(1200,500);node[61].choose=STOREFLAG;only=1;}
		else if((Point1.x>=480&&Point1.x<=490)&&(Point1.y>=454&&Point1.y<=464)) {Beep(1200,500);node[62].choose=STOREFLAG;only=1;}
		else if((Point1.x>=541&&Point1.x<=551)&&(Point1.y>=160&&Point1.y<=170)) {Beep(1200,500);node[63].choose=STOREFLAG;only=1;}
		else if((Point1.x>=548&&Point1.x<=558)&&(Point1.y>=359&&Point1.y<=369)) {Beep(1200,500);node[64].choose=STOREFLAG;only=1;}
		else if((Point1.x>=541&&Point1.x<=551)&&(Point1.y>=390&&Point1.y<=400)) {Beep(1200,500);node[65].choose=STOREFLAG;only=1;}
		else if((Point1.x>=543&&Point1.x<=553)&&(Point1.y>=432&&Point1.y<=442)) {Beep(1200,500);node[66].choose=STOREFLAG;only=1;}
		else if((Point1.x>=541&&Point1.x<=551)&&(Point1.y>=470&&Point1.y<=480)) {Beep(1200,500);node[67].choose=STOREFLAG;only=1;}
		else if((Point1.x>=510&&Point1.x<=520)&&(Point1.y>=591&&Point1.y<=601)) {Beep(1200,500);node[68].choose=STOREFLAG;only=1;}
		else if((Point1.x>=660&&Point1.x<=670)&&(Point1.y>=412&&Point1.y<=422)) {Beep(1200,500);node[69].choose=STOREFLAG;only=1;}
		else if((Point1.x>=625&&Point1.x<=635)&&(Point1.y>=510&&Point1.y<=520)) {Beep(1200,500);node[70].choose=STOREFLAG;only=1;}
		else if((Point1.x>=728&&Point1.x<=738)&&(Point1.y>=448&&Point1.y<=458)) {Beep(1200,500);node[71].choose=STOREFLAG;only=1;}
	}
}
/***************************************************************************************************************************************/
void CMyDlg::OnClientMenu() //客户标记菜单
{
	//显示客户旗
	ClientFlag();

	//识别客户地址并标记它的属性
	if	   ((Point1.x>=227&&Point1.x<=237)&&(Point1.y>=226&&Point1.y<=236))	{Beep(1200,500);node[51].choose=CLIENTFLAG;}
	else if((Point1.x>=227&&Point1.x<=237)&&(Point1.y>=297&&Point1.y<=307))	{Beep(1200,500);node[52].choose=CLIENTFLAG;}
	else if((Point1.x>=227&&Point1.x<=237)&&(Point1.y>=377&&Point1.y<=387)) {Beep(1200,500);node[53].choose=CLIENTFLAG;}
	else if((Point1.x>=233&&Point1.x<=243)&&(Point1.y>=458&&Point1.y<=468)) {Beep(1200,500);node[54].choose=CLIENTFLAG;}
	else if((Point1.x>=315&&Point1.x<=325)&&(Point1.y>=223&&Point1.y<=233)) {Beep(1200,500);node[55].choose=CLIENTFLAG;}
	else if((Point1.x>=317&&Point1.x<=327)&&(Point1.y>=460&&Point1.y<=470)) {Beep(1200,500);node[56].choose=CLIENTFLAG;}
	else if((Point1.x>=450&&Point1.x<=460)&&(Point1.y>=373&&Point1.y<=383)) {Beep(1200,500);node[57].choose=CLIENTFLAG;}
	else if((Point1.x>=510&&Point1.x<=520)&&(Point1.y>=35 &&Point1.y<=45 )) {Beep(1200,500);node[58].choose=CLIENTFLAG;}
	else if((Point1.x>=510&&Point1.x<=520)&&(Point1.y>=99 &&Point1.y<=109)) {Beep(1200,500);node[59].choose=CLIENTFLAG;}
	else if((Point1.x>=513&&Point1.x<=523)&&(Point1.y>=203&&Point1.y<=213)) {Beep(1200,500);node[60].choose=CLIENTFLAG;}
	else if((Point1.x>=510&&Point1.x<=520)&&(Point1.y>=290&&Point1.y<=300)) {Beep(1200,500);node[61].choose=CLIENTFLAG;}
	else if((Point1.x>=480&&Point1.x<=490)&&(Point1.y>=454&&Point1.y<=464)) {Beep(1200,500);node[62].choose=CLIENTFLAG;}
	else if((Point1.x>=541&&Point1.x<=551)&&(Point1.y>=160&&Point1.y<=170)) {Beep(1200,500);node[63].choose=CLIENTFLAG;}
	else if((Point1.x>=548&&Point1.x<=558)&&(Point1.y>=359&&Point1.y<=369)) {Beep(1200,500);node[64].choose=CLIENTFLAG;}
	else if((Point1.x>=541&&Point1.x<=551)&&(Point1.y>=390&&Point1.y<=400)) {Beep(1200,500);node[65].choose=CLIENTFLAG;}
	else if((Point1.x>=543&&Point1.x<=553)&&(Point1.y>=432&&Point1.y<=442)) {Beep(1200,500);node[66].choose=CLIENTFLAG;}
	else if((Point1.x>=541&&Point1.x<=551)&&(Point1.y>=470&&Point1.y<=480)) {Beep(1200,500);node[67].choose=CLIENTFLAG;}
	else if((Point1.x>=510&&Point1.x<=520)&&(Point1.y>=591&&Point1.y<=601)) {Beep(1200,500);node[68].choose=CLIENTFLAG;}
	else if((Point1.x>=660&&Point1.x<=670)&&(Point1.y>=412&&Point1.y<=422)) {Beep(1200,500);node[69].choose=CLIENTFLAG;}
	else if((Point1.x>=625&&Point1.x<=635)&&(Point1.y>=510&&Point1.y<=520)) {Beep(1200,500);node[70].choose=CLIENTFLAG;}
	else if((Point1.x>=728&&Point1.x<=738)&&(Point1.y>=448&&Point1.y<=458)) {Beep(1200,500);node[71].choose=CLIENTFLAG;}
}
/***************************************************************************************************************************************/
void CMyDlg::OnSmoothMenu() //流畅菜单响应
{
	//识别流畅路况并标记它的属性
	if     ((Point1.x>=219&&Point1.x<=225)&&(Point1.y>=231&&Point1.y<=271)) {Beep(300,500);road[1 ].traffic=SMOOTH;oncrowd=CheckOncrowd(1);if(plan==1) adjust=TRUE;Picture(219,231,225,271);}
	else if((Point1.x>=219&&Point1.x<=225)&&(Point1.y>=271&&Point1.y<=333)) {Beep(300,500);road[2 ].traffic=SMOOTH;oncrowd=CheckOncrowd(2);if(plan==1) adjust=TRUE;Picture(219,271,225,333);}
	else if((Point1.x>=219&&Point1.x<=224)&&(Point1.y>=333&&Point1.y<=416)) {Beep(300,500);road[3 ].traffic=SMOOTH;oncrowd=CheckOncrowd(3);if(plan==1) adjust=TRUE;Picture(219,333,224,416);}
	else if((Point1.x>=219&&Point1.x<=225)&&(Point1.y>=416&&Point1.y<=494)) {Beep(300,500);road[4 ].traffic=SMOOTH;oncrowd=CheckOncrowd(4);if(plan==1) adjust=TRUE;Picture(219,416,225,494);}
	else if((Point1.x>=224&&Point1.x<=306)&&(Point1.y>=268&&Point1.y<=274)) {Beep(300,500);road[5 ].traffic=SMOOTH;oncrowd=CheckOncrowd(5);if(plan==1) adjust=TRUE;Picture(224,268,306,274);}
	else if((Point1.x>=306&&Point1.x<=338)&&(Point1.y>=268&&Point1.y<=283)) {Beep(300,500);road[6 ].traffic=SMOOTH;oncrowd=CheckOncrowd(6);if(plan==1) adjust=TRUE;Picture(306,268,338,283);}
	else if((Point1.x>=222&&Point1.x<=338)&&(Point1.y>=330&&Point1.y<=336)) {Beep(300,500);road[7 ].traffic=SMOOTH;oncrowd=CheckOncrowd(7);if(plan==1) adjust=TRUE;Picture(222,330,338,336);}
	else if((Point1.x>=222&&Point1.x<=338)&&(Point1.y>=413&&Point1.y<=419)) {Beep(300,500);road[8 ].traffic=SMOOTH;oncrowd=CheckOncrowd(8);if(plan==1) adjust=TRUE;Picture(222,413,338,419);}
	else if((Point1.x>=222&&Point1.x<=340)&&(Point1.y>=491&&Point1.y<=497)) {Beep(300,500);road[9 ].traffic=SMOOTH;oncrowd=CheckOncrowd(9);if(plan==1) adjust=TRUE;Picture(222,491,340,497);}
	else if((Point1.x>=337&&Point1.x<=343)&&(Point1.y>=228&&Point1.y<=280)) {Beep(300,500);road[10].traffic=SMOOTH;oncrowd=CheckOncrowd(10);if(plan==1) adjust=TRUE;Picture(337,228,343,280);}
	else if((Point1.x>=337&&Point1.x<=343)&&(Point1.y>=277&&Point1.y<=333)) {Beep(300,500);road[11].traffic=SMOOTH;oncrowd=CheckOncrowd(11);if(plan==1) adjust=TRUE;Picture(337,277,343,333);}
	else if((Point1.x>=337&&Point1.x<=342)&&(Point1.y>=333&&Point1.y<=414)) {Beep(300,500);road[12].traffic=SMOOTH;oncrowd=CheckOncrowd(12);if(plan==1) adjust=TRUE;Picture(337,333,342,414);}
	else if((Point1.x>=337&&Point1.x<=343)&&(Point1.y>=414&&Point1.y<=494)) {Beep(300,500);road[13].traffic=SMOOTH;oncrowd=CheckOncrowd(13);if(plan==1) adjust=TRUE;Picture(337,414,343,494);}
	else if((Point1.x>=340&&Point1.x<=384)&&(Point1.y>=277&&Point1.y<=282)) {Beep(300,500);road[14].traffic=SMOOTH;oncrowd=CheckOncrowd(14);if(plan==1) adjust=TRUE;Picture(340,277,384,282);}
	else if((Point1.x>=340&&Point1.x<=384)&&(Point1.y>=332&&Point1.y<=336)) {Beep(300,500);road[15].traffic=SMOOTH;oncrowd=CheckOncrowd(15);if(plan==1) adjust=TRUE;Picture(340,332,384,336);}
	else if((Point1.x>=338&&Point1.x<=426)&&(Point1.y>=413&&Point1.y<=418)) {Beep(300,500);road[16].traffic=SMOOTH;oncrowd=CheckOncrowd(16);if(plan==1) adjust=TRUE;Picture(338,413,426,418);}
	else if((Point1.x>=340&&Point1.x<=426)&&(Point1.y>=491&&Point1.y<=497)) {Beep(300,500);road[17].traffic=SMOOTH;oncrowd=CheckOncrowd(17);if(plan==1) adjust=TRUE;Picture(340,491,426,497);}
	else if((Point1.x>=381&&Point1.x<=387)&&(Point1.y>=280&&Point1.y<=333)) {Beep(300,500);road[18].traffic=SMOOTH;oncrowd=CheckOncrowd(18);if(plan==1) adjust=TRUE;Picture(381,280,387,333);}
	else if((Point1.x>=423&&Point1.x<=429)&&(Point1.y>=332&&Point1.y<=413)) {Beep(300,500);road[19].traffic=SMOOTH;oncrowd=CheckOncrowd(19);if(plan==1) adjust=TRUE;Picture(423,332,429,413);}
	else if((Point1.x>=423&&Point1.x<=429)&&(Point1.y>=413&&Point1.y<=493)) {Beep(300,500);road[20].traffic=SMOOTH;oncrowd=CheckOncrowd(20);if(plan==1) adjust=TRUE;Picture(423,413,429,493);}
	else if((Point1.x>=384&&Point1.x<=476)&&(Point1.y>=277&&Point1.y<=281)) {Beep(300,500);road[21].traffic=SMOOTH;oncrowd=CheckOncrowd(21);if(plan==1) adjust=TRUE;Picture(384,277,476,281);}
	else if((Point1.x>=384&&Point1.x<=426)&&(Point1.y>=330&&Point1.y<=336)) {Beep(300,500);road[22].traffic=SMOOTH;oncrowd=CheckOncrowd(22);if(plan==1) adjust=TRUE;Picture(384,330,426,336);}
	else if((Point1.x>=426&&Point1.x<=476)&&(Point1.y>=330&&Point1.y<=335)) {Beep(300,500);road[23].traffic=SMOOTH;oncrowd=CheckOncrowd(23);if(plan==1) adjust=TRUE;Picture(426,330,476,335);}
	else if((Point1.x>=426&&Point1.x<=476)&&(Point1.y>=412&&Point1.y<=416)) {Beep(300,500);road[24].traffic=SMOOTH;oncrowd=CheckOncrowd(24);if(plan==1) adjust=TRUE;Picture(426,412,476,416);}
	else if((Point1.x>=426&&Point1.x<=476)&&(Point1.y>=490&&Point1.y<=496)) {Beep(300,500);road[25].traffic=SMOOTH;oncrowd=CheckOncrowd(25);if(plan==1) adjust=TRUE;Picture(426,490,476,496);}
	else if((Point1.x>=473&&Point1.x<=477)&&(Point1.y>=218&&Point1.y<=277)) {Beep(300,500);road[26].traffic=SMOOTH;oncrowd=CheckOncrowd(26);if(plan==1) adjust=TRUE;Picture(473,218,477,277);}
	else if((Point1.x>=473&&Point1.x<=477)&&(Point1.y>=277&&Point1.y<=330)) {Beep(300,500);road[27].traffic=SMOOTH;oncrowd=CheckOncrowd(27);if(plan==1) adjust=TRUE;Picture(473,277,477,330);}
	else if((Point1.x>=473&&Point1.x<=477)&&(Point1.y>=330&&Point1.y<=413)) {Beep(300,500);road[28].traffic=SMOOTH;oncrowd=CheckOncrowd(28);if(plan==1) adjust=TRUE;Picture(473,330,477,413);}
	else if((Point1.x>=473&&Point1.x<=477)&&(Point1.y>=413&&Point1.y<=492)) {Beep(300,500);road[29].traffic=SMOOTH;oncrowd=CheckOncrowd(29);if(plan==1) adjust=TRUE;Picture(473,413,477,492);}
	else if((Point1.x>=477&&Point1.x<=506)&&(Point1.y>=215&&Point1.y<=222)) {Beep(300,500);road[30].traffic=SMOOTH;oncrowd=CheckOncrowd(30);if(plan==1) adjust=TRUE;Picture(477,215,506,222);}
	else if((Point1.x>=476&&Point1.x<=493)&&(Point1.y>=274&&Point1.y<=288)) {Beep(300,500);road[31].traffic=SMOOTH;oncrowd=CheckOncrowd(31);if(plan==1) adjust=TRUE;Picture(476,274,493,288);}
	else if((Point1.x>=493&&Point1.x<=503)&&(Point1.y>=265&&Point1.y<=285)) {Beep(300,500);road[32].traffic=SMOOTH;oncrowd=CheckOncrowd(32);if(plan==1) adjust=TRUE;Picture(493,265,503,285);}
   	else if((Point1.x>=477&&Point1.x<=493)&&(Point1.y>=285&&Point1.y<=328)) {Beep(300,500);road[33].traffic=SMOOTH;oncrowd=CheckOncrowd(33);if(plan==1) adjust=TRUE;Picture(477,285,493,328);}
    else if((Point1.x>=476&&Point1.x<=505)&&(Point1.y>=327&&Point1.y<=333)) {Beep(300,500);road[34].traffic=SMOOTH;oncrowd=CheckOncrowd(34);if(plan==1) adjust=TRUE;Picture(476,327,505,333);}
	else if((Point1.x>=476&&Point1.x<=505)&&(Point1.y>=410&&Point1.y<=416)) {Beep(300,500);road[35].traffic=SMOOTH;oncrowd=CheckOncrowd(35);if(plan==1) adjust=TRUE;Picture(476,410,505,416);}
	else if((Point1.x>=476&&Point1.x<=538)&&(Point1.y>=490&&Point1.y<=495)) {Beep(300,500);road[36].traffic=SMOOTH;oncrowd=CheckOncrowd(36);if(plan==1) adjust=TRUE;Picture(476,490,538,495);}
	else if((Point1.x>=502&&Point1.x<=508)&&(Point1.y>=40 &&Point1.y<=68 )) {Beep(300,500);road[37].traffic=SMOOTH;oncrowd=CheckOncrowd(37);if(plan==1) adjust=TRUE;Picture(502,40,508,68  );}
	else if((Point1.x>=502&&Point1.x<=508)&&(Point1.y>=68 &&Point1.y<=123)) {Beep(300,500);road[38].traffic=SMOOTH;oncrowd=CheckOncrowd(38);if(plan==1) adjust=TRUE;Picture(502,68,508,123 );}
	else if((Point1.x>=504&&Point1.x<=508)&&(Point1.y>=123&&Point1.y<=218)) {Beep(300,500);road[39].traffic=SMOOTH;oncrowd=CheckOncrowd(39);if(plan==1) adjust=TRUE;Picture(504,123,508,218);}
	else if((Point1.x>=503&&Point1.x<=508)&&(Point1.y>=218&&Point1.y<=265)) {Beep(300,500);road[40].traffic=SMOOTH;oncrowd=CheckOncrowd(40);if(plan==1) adjust=TRUE;Picture(503,218,508,265);}
	else if((Point1.x>=503&&Point1.x<=508)&&(Point1.y>=265&&Point1.y<=330)) {Beep(300,500);road[41].traffic=SMOOTH;oncrowd=CheckOncrowd(41);if(plan==1) adjust=TRUE;Picture(503,265,508,330);}
	else if((Point1.x>=503&&Point1.x<=508)&&(Point1.y>=330&&Point1.y<=413)) {Beep(300,500);road[42].traffic=SMOOTH;oncrowd=CheckOncrowd(42);if(plan==1) adjust=TRUE;Picture(503,330,508,413);}
	else if((Point1.x>=505&&Point1.x<=538)&&(Point1.y>=65 &&Point1.y<=71 )) {Beep(300,500);road[43].traffic=SMOOTH;oncrowd=CheckOncrowd(43);if(plan==1) adjust=TRUE;Picture(505,65,538,71  );}
	else if((Point1.x>=505&&Point1.x<=538)&&(Point1.y>=120&&Point1.y<=126)) {Beep(300,500);road[44].traffic=SMOOTH;oncrowd=CheckOncrowd(44);if(plan==1) adjust=TRUE;Picture(505,120,538,126);}
	else if((Point1.x>=505&&Point1.x<=529)&&(Point1.y>=215&&Point1.y<=221)) {Beep(300,500);road[45].traffic=SMOOTH;oncrowd=CheckOncrowd(45);if(plan==1) adjust=TRUE;Picture(505,215,529,221);}
	else if((Point1.x>=505&&Point1.x<=538)&&(Point1.y>=327&&Point1.y<=333)) {Beep(300,500);road[46].traffic=SMOOTH;oncrowd=CheckOncrowd(46);if(plan==1) adjust=TRUE;Picture(505,327,538,333);}
	else if((Point1.x>=505&&Point1.x<=538)&&(Point1.y>=410&&Point1.y<=416)) {Beep(300,500);road[47].traffic=SMOOTH;oncrowd=CheckOncrowd(47);if(plan==1) adjust=TRUE;Picture(505,410,538,416);}
	else if((Point1.x>=535&&Point1.x<=539)&&(Point1.y>=68 &&Point1.y<=123)) {Beep(300,500);road[48].traffic=SMOOTH;oncrowd=CheckOncrowd(48);if(plan==1) adjust=TRUE;Picture(535,68,539,123 );}
	else if((Point1.x>=535&&Point1.x<=541)&&(Point1.y>=123&&Point1.y<=218)) {Beep(300,500);road[49].traffic=SMOOTH;oncrowd=CheckOncrowd(49);if(plan==1) adjust=TRUE;Picture(535,123,541,218);}
	else if((Point1.x>=535&&Point1.x<=539)&&(Point1.y>=218&&Point1.y<=286)) {Beep(300,500);road[50].traffic=SMOOTH;oncrowd=CheckOncrowd(50);if(plan==1) adjust=TRUE;Picture(535,218,539,286);}
	else if((Point1.x>=535&&Point1.x<=539)&&(Point1.y>=286&&Point1.y<=330)) {Beep(300,500);road[51].traffic=SMOOTH;oncrowd=CheckOncrowd(51);if(plan==1) adjust=TRUE;Picture(535,286,539,330);}		
	else if((Point1.x>=535&&Point1.x<=540)&&(Point1.y>=330&&Point1.y<=413)) {Beep(300,500);road[52].traffic=SMOOTH;oncrowd=CheckOncrowd(52);if(plan==1) adjust=TRUE;Picture(535,330,540,413);}
	else if((Point1.x>=535&&Point1.x<=541)&&(Point1.y>=413&&Point1.y<=491)) {Beep(300,500);road[53].traffic=SMOOTH;oncrowd=CheckOncrowd(53);if(plan==1) adjust=TRUE;Picture(535,413,541,491);}
	else if((Point1.x>=535&&Point1.x<=541)&&(Point1.y>=491&&Point1.y<=596)) {Beep(300,500);road[54].traffic=SMOOTH;oncrowd=CheckOncrowd(54);if(plan==1) adjust=TRUE;Picture(535,491,541,596);}
	else if((Point1.x>=538&&Point1.x<=612)&&(Point1.y>=327&&Point1.y<=331)) {Beep(300,500);road[55].traffic=SMOOTH;oncrowd=CheckOncrowd(55);if(plan==1) adjust=TRUE;Picture(538,327,612,331);}
	else if((Point1.x>=538&&Point1.x<=584)&&(Point1.y>=410&&Point1.y<=416)) {Beep(300,500);road[56].traffic=SMOOTH;oncrowd=CheckOncrowd(56);if(plan==1) adjust=TRUE;Picture(538,410,584,416);}
	else if((Point1.x>=584&&Point1.x<=648)&&(Point1.y>=412&&Point1.y<=424)) {Beep(300,500);road[57].traffic=SMOOTH;oncrowd=CheckOncrowd(57);if(plan==1) adjust=TRUE;Picture(584,412,648,424);}
	else if((Point1.x>=541&&Point1.x<=595)&&(Point1.y>=491&&Point1.y<=506)) {Beep(300,500);road[58].traffic=SMOOTH;oncrowd=CheckOncrowd(58);if(plan==1) adjust=TRUE;Picture(541,491,595,506);}
	else if((Point1.x>=595&&Point1.x<=642)&&(Point1.y>=503&&Point1.y<=511)) {Beep(300,500);road[59].traffic=SMOOTH;oncrowd=CheckOncrowd(59);if(plan==1) adjust=TRUE;Picture(595,503,642,511);}
	else if((Point1.x>=610&&Point1.x<=631)&&(Point1.y>=330&&Point1.y<=363)) {Beep(300,500);road[60].traffic=SMOOTH;oncrowd=CheckOncrowd(60);if(plan==1) adjust=TRUE;Picture(610,330,631,363);}
	else if((Point1.x>=631&&Point1.x<=651)&&(Point1.y>=363&&Point1.y<=386)) {Beep(300,500);road[61].traffic=SMOOTH;oncrowd=CheckOncrowd(61);if(plan==1) adjust=TRUE;Picture(631,363,651,386);}
	else if((Point1.x>=647&&Point1.x<=653)&&(Point1.y>=385&&Point1.y<=424)) {Beep(300,500);road[62].traffic=SMOOTH;oncrowd=CheckOncrowd(62);if(plan==1) adjust=TRUE;Picture(648,385,651,424);}
	else if((Point1.x>=646&&Point1.x<=651)&&(Point1.y>=424&&Point1.y<=456)) {Beep(300,500);road[63].traffic=SMOOTH;oncrowd=CheckOncrowd(63);if(plan==1) adjust=TRUE;Picture(646,424,651,456);}
	else if((Point1.x>=645&&Point1.x<=648)&&(Point1.y>=456&&Point1.y<=469)) {Beep(300,500);road[64].traffic=SMOOTH;oncrowd=CheckOncrowd(64);if(plan==1) adjust=TRUE;Picture(645,456,651,469);}
	else if((Point1.x>=640&&Point1.x<=647)&&(Point1.y>=469&&Point1.y<=508)) {Beep(300,500);road[65].traffic=SMOOTH;oncrowd=CheckOncrowd(65);if(plan==1) adjust=TRUE;Picture(640,469,647,508);}
	else if((Point1.x>=653&&Point1.x<=710)&&(Point1.y>=385&&Point1.y<=408)) {Beep(300,500);road[66].traffic=SMOOTH;oncrowd=CheckOncrowd(66);if(plan==1) adjust=TRUE;Picture(653,385,710,408);}
	else if((Point1.x>=650&&Point1.x<=665)&&(Point1.y>=422&&Point1.y<=427)) {Beep(300,500);road[67].traffic=SMOOTH;oncrowd=CheckOncrowd(67);if(plan==1) adjust=TRUE;Picture(650,422,665,427);}
	else if((Point1.x>=642&&Point1.x<=678)&&(Point1.y>=506&&Point1.y<=515)) {Beep(300,500);road[68].traffic=SMOOTH;oncrowd=CheckOncrowd(68);if(plan==1) adjust=TRUE;Picture(642,506,678,515);}
	else if((Point1.x>=678&&Point1.x<=701)&&(Point1.y>=509&&Point1.y<=515)) {Beep(300,500);road[69].traffic=SMOOTH;oncrowd=CheckOncrowd(69);if(plan==1) adjust=TRUE;Picture(678,509,701,515);}
	else if((Point1.x>=701&&Point1.x<=712)&&(Point1.y>=515&&Point1.y<=529)) {Beep(300,500);road[70].traffic=SMOOTH;oncrowd=CheckOncrowd(70);if(plan==1) adjust=TRUE;Picture(698,515,713,525);}
	else if((Point1.x>=710&&Point1.x<=723)&&(Point1.y>=408&&Point1.y<=432)) {Beep(300,500);road[71].traffic=SMOOTH;oncrowd=CheckOncrowd(71);if(plan==1) adjust=TRUE;Picture(710,408,723,432);}
	else if((Point1.x>=720&&Point1.x<=725)&&(Point1.y>=433&&Point1.y<=478)) {Beep(300,500);road[72].traffic=SMOOTH;oncrowd=CheckOncrowd(72);if(plan==1) adjust=TRUE;Picture(720,433,725,478);}
	else if((Point1.x>=711&&Point1.x<=722)&&(Point1.y>=478&&Point1.y<=530)) {Beep(300,500);road[73].traffic=SMOOTH;oncrowd=CheckOncrowd(73);if(plan==1) adjust=TRUE;Picture(711,478,722,530);}
	else if((Point1.x>=529&&Point1.x<=538)&&(Point1.y>=215&&Point1.y<=221)) {Beep(300,500);road[74].traffic=SMOOTH;oncrowd=CheckOncrowd(74);if(plan==1) adjust=TRUE;Picture(529,215,538,221);}
	else if((Point1.x>=508&&Point1.x<=535)&&(Point1.y>=223&&Point1.y<=265)) {Beep(300,500);road[75].traffic=SMOOTH;oncrowd=CheckOncrowd(75);if(plan==1) adjust=TRUE;Picture(508,223,535,265);}

	if(m_MinRadio==1 && ing==TRUE) //最短时间方案
	{
		ChangeCircle(); //动态规划新Head
	
		//文本显示
		CalculateHead();
		Statistic();
	}
}
/***************************************************************************************************************************************/
void CMyDlg::OnCrowdMenu() //拥挤菜单响应
{

	//识别拥挤路况并标记它的属性
	if     ((Point1.x>=219&&Point1.x<=225)&&(Point1.y>=231&&Point1.y<=271)) {Beep(300,500);road[1 ].traffic=CROWD;oncrowd=CheckOncrowd(1);if(ing==TRUE) road[1 ].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=219&&Point1.x<=225)&&(Point1.y>=271&&Point1.y<=333)) {Beep(300,500);road[2 ].traffic=CROWD;oncrowd=CheckOncrowd(2);if(ing==TRUE) road[2 ].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=219&&Point1.x<=224)&&(Point1.y>=333&&Point1.y<=416)) {Beep(300,500);road[3 ].traffic=CROWD;oncrowd=CheckOncrowd(3);if(ing==TRUE) road[3 ].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=219&&Point1.x<=225)&&(Point1.y>=416&&Point1.y<=494)) {Beep(300,500);road[4 ].traffic=CROWD;oncrowd=CheckOncrowd(4);if(ing==TRUE) road[4 ].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=224&&Point1.x<=306)&&(Point1.y>=268&&Point1.y<=274)) {Beep(300,500);road[5 ].traffic=CROWD;oncrowd=CheckOncrowd(5);if(ing==TRUE) road[5 ].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=308&&Point1.x<=338)&&(Point1.y>=268&&Point1.y<=283)) {Beep(300,500);road[6 ].traffic=CROWD;oncrowd=CheckOncrowd(6);if(ing==TRUE) road[6 ].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=222&&Point1.x<=338)&&(Point1.y>=330&&Point1.y<=336)) {Beep(300,500);road[7 ].traffic=CROWD;oncrowd=CheckOncrowd(7);if(ing==TRUE) road[7 ].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=222&&Point1.x<=338)&&(Point1.y>=413&&Point1.y<=419)) {Beep(300,500);road[8 ].traffic=CROWD;oncrowd=CheckOncrowd(8);if(ing==TRUE) road[8 ].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=222&&Point1.x<=340)&&(Point1.y>=491&&Point1.y<=497)) {Beep(300,500);road[9 ].traffic=CROWD;oncrowd=CheckOncrowd(9);if(ing==TRUE) road[9 ].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=337&&Point1.x<=343)&&(Point1.y>=228&&Point1.y<=280)) {Beep(300,500);road[10].traffic=CROWD;oncrowd=CheckOncrowd(10);if(ing==TRUE) road[10].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=337&&Point1.x<=343)&&(Point1.y>=277&&Point1.y<=333)) {Beep(300,500);road[11].traffic=CROWD;oncrowd=CheckOncrowd(11);if(ing==TRUE) road[11].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=337&&Point1.x<=342)&&(Point1.y>=333&&Point1.y<=414)) {Beep(300,500);road[12].traffic=CROWD;oncrowd=CheckOncrowd(12);if(ing==TRUE) road[12].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=337&&Point1.x<=343)&&(Point1.y>=414&&Point1.y<=494)) {Beep(300,500);road[13].traffic=CROWD;oncrowd=CheckOncrowd(13);if(ing==TRUE) road[13].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=340&&Point1.x<=384)&&(Point1.y>=277&&Point1.y<=282)) {Beep(300,500);road[14].traffic=CROWD;oncrowd=CheckOncrowd(14);if(ing==TRUE) road[14].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=340&&Point1.x<=384)&&(Point1.y>=332&&Point1.y<=336)) {Beep(300,500);road[15].traffic=CROWD;oncrowd=CheckOncrowd(15);if(ing==TRUE) road[15].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=338&&Point1.x<=426)&&(Point1.y>=413&&Point1.y<=418)) {Beep(300,500);road[16].traffic=CROWD;oncrowd=CheckOncrowd(16);if(ing==TRUE) road[16].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=340&&Point1.x<=426)&&(Point1.y>=491&&Point1.y<=497)) {Beep(300,500);road[17].traffic=CROWD;oncrowd=CheckOncrowd(17);if(ing==TRUE) road[17].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=381&&Point1.x<=387)&&(Point1.y>=280&&Point1.y<=333)) {Beep(300,500);road[18].traffic=CROWD;oncrowd=CheckOncrowd(18);if(ing==TRUE) road[18].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=423&&Point1.x<=429)&&(Point1.y>=332&&Point1.y<=413)) {Beep(300,500);road[19].traffic=CROWD;oncrowd=CheckOncrowd(19);if(ing==TRUE) road[19].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=423&&Point1.x<=429)&&(Point1.y>=413&&Point1.y<=493)) {Beep(300,500);road[20].traffic=CROWD;oncrowd=CheckOncrowd(20);if(ing==TRUE) road[20].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=384&&Point1.x<=476)&&(Point1.y>=277&&Point1.y<=281)) {Beep(300,500);road[21].traffic=CROWD;oncrowd=CheckOncrowd(21);if(ing==TRUE) road[21].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=384&&Point1.x<=426)&&(Point1.y>=330&&Point1.y<=336)) {Beep(300,500);road[22].traffic=CROWD;oncrowd=CheckOncrowd(22);if(ing==TRUE) road[22].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=426&&Point1.x<=476)&&(Point1.y>=330&&Point1.y<=335)) {Beep(300,500);road[23].traffic=CROWD;oncrowd=CheckOncrowd(23);if(ing==TRUE) road[23].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=426&&Point1.x<=476)&&(Point1.y>=412&&Point1.y<=416)) {Beep(300,500);road[24].traffic=CROWD;oncrowd=CheckOncrowd(24);if(ing==TRUE) road[24].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=426&&Point1.x<=476)&&(Point1.y>=490&&Point1.y<=496)) {Beep(300,500);road[25].traffic=CROWD;oncrowd=CheckOncrowd(25);if(ing==TRUE) road[25].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=473&&Point1.x<=477)&&(Point1.y>=218&&Point1.y<=277)) {Beep(300,500);road[26].traffic=CROWD;oncrowd=CheckOncrowd(26);if(ing==TRUE) road[26].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=473&&Point1.x<=477)&&(Point1.y>=277&&Point1.y<=330)) {Beep(300,500);road[27].traffic=CROWD;oncrowd=CheckOncrowd(27);if(ing==TRUE) road[27].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=473&&Point1.x<=477)&&(Point1.y>=330&&Point1.y<=413)) {Beep(300,500);road[28].traffic=CROWD;oncrowd=CheckOncrowd(28);if(ing==TRUE) road[28].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=473&&Point1.x<=477)&&(Point1.y>=413&&Point1.y<=492)) {Beep(300,500);road[29].traffic=CROWD;oncrowd=CheckOncrowd(29);if(ing==TRUE) road[29].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=477&&Point1.x<=506)&&(Point1.y>=215&&Point1.y<=222)) {Beep(300,500);road[30].traffic=CROWD;oncrowd=CheckOncrowd(30);if(ing==TRUE) road[30].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=476&&Point1.x<=493)&&(Point1.y>=274&&Point1.y<=288)) {Beep(300,500);road[31].traffic=CROWD;oncrowd=CheckOncrowd(31);if(ing==TRUE) road[31].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=493&&Point1.x<=503)&&(Point1.y>=265&&Point1.y<=285)) {Beep(300,500);road[32].traffic=CROWD;oncrowd=CheckOncrowd(32);if(ing==TRUE) road[32].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
   	else if((Point1.x>=477&&Point1.x<=493)&&(Point1.y>=285&&Point1.y<=328)) {Beep(300,500);road[33].traffic=CROWD;oncrowd=CheckOncrowd(33);if(ing==TRUE) road[33].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
    else if((Point1.x>=476&&Point1.x<=505)&&(Point1.y>=327&&Point1.y<=333)) {Beep(300,500);road[34].traffic=CROWD;oncrowd=CheckOncrowd(34);if(ing==TRUE) road[34].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=476&&Point1.x<=505)&&(Point1.y>=410&&Point1.y<=416)) {Beep(300,500);road[35].traffic=CROWD;oncrowd=CheckOncrowd(35);if(ing==TRUE) road[35].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=476&&Point1.x<=538)&&(Point1.y>=490&&Point1.y<=495)) {Beep(300,500);road[36].traffic=CROWD;oncrowd=CheckOncrowd(36);if(ing==TRUE) road[36].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=502&&Point1.x<=508)&&(Point1.y>=40 &&Point1.y<=68 )) {Beep(300,500);road[37].traffic=CROWD;oncrowd=CheckOncrowd(37);if(ing==TRUE) road[37].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=502&&Point1.x<=508)&&(Point1.y>=68 &&Point1.y<=123)) {Beep(300,500);road[38].traffic=CROWD;oncrowd=CheckOncrowd(38);if(ing==TRUE) road[38].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=504&&Point1.x<=508)&&(Point1.y>=123&&Point1.y<=218)) {Beep(300,500);road[39].traffic=CROWD;oncrowd=CheckOncrowd(39);if(ing==TRUE) road[39].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=503&&Point1.x<=508)&&(Point1.y>=218&&Point1.y<=265)) {Beep(300,500);road[40].traffic=CROWD;oncrowd=CheckOncrowd(40);if(ing==TRUE) road[40].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=503&&Point1.x<=508)&&(Point1.y>=265&&Point1.y<=330)) {Beep(300,500);road[41].traffic=CROWD;oncrowd=CheckOncrowd(41);if(ing==TRUE) road[41].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=503&&Point1.x<=508)&&(Point1.y>=330&&Point1.y<=413)) {Beep(300,500);road[42].traffic=CROWD;oncrowd=CheckOncrowd(42);if(ing==TRUE) road[42].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=505&&Point1.x<=538)&&(Point1.y>=65 &&Point1.y<=71 )) {Beep(300,500);road[43].traffic=CROWD;oncrowd=CheckOncrowd(43);if(ing==TRUE) road[43].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=505&&Point1.x<=538)&&(Point1.y>=120&&Point1.y<=126)) {Beep(300,500);road[44].traffic=CROWD;oncrowd=CheckOncrowd(44);if(ing==TRUE) road[44].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=505&&Point1.x<=529)&&(Point1.y>=215&&Point1.y<=221)) {Beep(300,500);road[45].traffic=CROWD;oncrowd=CheckOncrowd(45);if(ing==TRUE) road[45].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=505&&Point1.x<=538)&&(Point1.y>=327&&Point1.y<=333)) {Beep(300,500);road[46].traffic=CROWD;oncrowd=CheckOncrowd(46);if(ing==TRUE) road[46].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=505&&Point1.x<=538)&&(Point1.y>=410&&Point1.y<=416)) {Beep(300,500);road[47].traffic=CROWD;oncrowd=CheckOncrowd(47);if(ing==TRUE) road[47].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=535&&Point1.x<=539)&&(Point1.y>=68 &&Point1.y<=123)) {Beep(300,500);road[48].traffic=CROWD;oncrowd=CheckOncrowd(48);if(ing==TRUE) road[48].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=535&&Point1.x<=541)&&(Point1.y>=123&&Point1.y<=218)) {Beep(300,500);road[49].traffic=CROWD;oncrowd=CheckOncrowd(49);if(ing==TRUE) road[49].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=535&&Point1.x<=539)&&(Point1.y>=218&&Point1.y<=286)) {Beep(300,500);road[50].traffic=CROWD;oncrowd=CheckOncrowd(50);if(ing==TRUE) road[50].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=535&&Point1.x<=539)&&(Point1.y>=286&&Point1.y<=330)) {Beep(300,500);road[51].traffic=CROWD;oncrowd=CheckOncrowd(51);if(ing==TRUE) road[51].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=535&&Point1.x<=540)&&(Point1.y>=330&&Point1.y<=413)) {Beep(300,500);road[52].traffic=CROWD;oncrowd=CheckOncrowd(52);if(ing==TRUE) road[52].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=535&&Point1.x<=541)&&(Point1.y>=413&&Point1.y<=491)) {Beep(300,500);road[53].traffic=CROWD;oncrowd=CheckOncrowd(53);if(ing==TRUE) road[53].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=535&&Point1.x<=541)&&(Point1.y>=491&&Point1.y<=596)) {Beep(300,500);road[54].traffic=CROWD;oncrowd=CheckOncrowd(54);if(ing==TRUE) road[54].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=538&&Point1.x<=612)&&(Point1.y>=327&&Point1.y<=331)) {Beep(300,500);road[55].traffic=CROWD;oncrowd=CheckOncrowd(55);if(ing==TRUE) road[55].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=538&&Point1.x<=584)&&(Point1.y>=410&&Point1.y<=416)) {Beep(300,500);road[56].traffic=CROWD;oncrowd=CheckOncrowd(56);if(ing==TRUE) road[56].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=584&&Point1.x<=648)&&(Point1.y>=412&&Point1.y<=424)) {Beep(300,500);road[57].traffic=CROWD;oncrowd=CheckOncrowd(57);if(ing==TRUE) road[57].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=541&&Point1.x<=595)&&(Point1.y>=491&&Point1.y<=506)) {Beep(300,500);road[58].traffic=CROWD;oncrowd=CheckOncrowd(58);if(ing==TRUE) road[58].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=595&&Point1.x<=642)&&(Point1.y>=503&&Point1.y<=511)) {Beep(300,500);road[59].traffic=CROWD;oncrowd=CheckOncrowd(59);if(ing==TRUE) road[59].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=610&&Point1.x<=631)&&(Point1.y>=330&&Point1.y<=363)) {Beep(300,500);road[60].traffic=CROWD;oncrowd=CheckOncrowd(60);if(ing==TRUE) road[60].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=631&&Point1.x<=651)&&(Point1.y>=363&&Point1.y<=386)) {Beep(300,500);road[61].traffic=CROWD;oncrowd=CheckOncrowd(61);if(ing==TRUE) road[61].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=647&&Point1.x<=653)&&(Point1.y>=385&&Point1.y<=424)) {Beep(300,500);road[62].traffic=CROWD;oncrowd=CheckOncrowd(62);if(ing==TRUE) road[62].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=646&&Point1.x<=651)&&(Point1.y>=424&&Point1.y<=456)) {Beep(300,500);road[63].traffic=CROWD;oncrowd=CheckOncrowd(63);if(ing==TRUE) road[63].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=645&&Point1.x<=648)&&(Point1.y>=456&&Point1.y<=469)) {Beep(300,500);road[64].traffic=CROWD;oncrowd=CheckOncrowd(64);if(ing==TRUE) road[64].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=640&&Point1.x<=647)&&(Point1.y>=469&&Point1.y<=508)) {Beep(300,500);road[65].traffic=CROWD;oncrowd=CheckOncrowd(65);if(ing==TRUE) road[65].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=653&&Point1.x<=710)&&(Point1.y>=385&&Point1.y<=408)) {Beep(300,500);road[66].traffic=CROWD;oncrowd=CheckOncrowd(66);if(ing==TRUE) road[66].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=650&&Point1.x<=665)&&(Point1.y>=422&&Point1.y<=427)) {Beep(300,500);road[67].traffic=CROWD;oncrowd=CheckOncrowd(67);if(ing==TRUE) road[67].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=642&&Point1.x<=678)&&(Point1.y>=506&&Point1.y<=515)) {Beep(300,500);road[68].traffic=CROWD;oncrowd=CheckOncrowd(68);if(ing==TRUE) road[68].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=678&&Point1.x<=701)&&(Point1.y>=509&&Point1.y<=515)) {Beep(300,500);road[69].traffic=CROWD;oncrowd=CheckOncrowd(69);if(ing==TRUE) road[69].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=701&&Point1.x<=712)&&(Point1.y>=515&&Point1.y<=529)) {Beep(300,500);road[70].traffic=CROWD;oncrowd=CheckOncrowd(70);if(ing==TRUE) road[70].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=710&&Point1.x<=723)&&(Point1.y>=408&&Point1.y<=432)) {Beep(300,500);road[71].traffic=CROWD;oncrowd=CheckOncrowd(71);if(ing==TRUE) road[71].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=720&&Point1.x<=725)&&(Point1.y>=433&&Point1.y<=478)) {Beep(300,500);road[72].traffic=CROWD;oncrowd=CheckOncrowd(72);if(ing==TRUE) road[72].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=711&&Point1.x<=722)&&(Point1.y>=478&&Point1.y<=530)) {Beep(300,500);road[73].traffic=CROWD;oncrowd=CheckOncrowd(73);if(ing==TRUE) road[73].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=529&&Point1.x<=538)&&(Point1.y>=215&&Point1.y<=221)) {Beep(300,500);road[74].traffic=CROWD;oncrowd=CheckOncrowd(74);if(ing==TRUE) road[74].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}
	else if((Point1.x>=508&&Point1.x<=535)&&(Point1.y>=223&&Point1.y<=265)) {Beep(300,500);road[75].traffic=CROWD;oncrowd=CheckOncrowd(75);if(ing==TRUE) road[75].then=TRUE;if(plan==1) adjust=TRUE;touch=OUTDOOR;}

	if(m_MinRadio == 1 && ing == TRUE) //最短时间方案
	{
		ChangeCircle(); //动态规划新Head
	
		//文本显示
		CalculateHead();
		Statistic();			
		
		return;	
	}
}
/***************************************************************************************************************************************/
void CMyDlg::OnTestButton() //测试按钮响应
{
	if(timer == TRUE) //中途按键则删除定时器
	{
		timer=FALSE;
		KillTimer(1);
	}
	
	//审核输入是否合法
	UpdateData(TRUE);
	//需求量
	if(CheckBlank(m_NeedEdit) == FALSE)
	{
		MessageBox("您还未输入需求量!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	if(CheckNum(m_NeedEdit) == FALSE)
	{
		MessageBox("请输入有效需求量!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	if(atof(m_NeedEdit) > 12)
	{
		MessageBox("最大载重不超过12吨!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	//燃油费
	if(CheckBlank(m_FuelEdit) == FALSE)
	{
		MessageBox("您还未输入燃油费!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	if(CheckNum(m_FuelEdit) == FALSE)
	{
		MessageBox("请输入有效燃油费!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	if(atof(m_FuelEdit) > 10)
	{
		MessageBox("燃油费不超过10元/升!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	//其余费
	if(CheckBlank(m_OtherEdit) == FALSE)
	{
		MessageBox("您还未输入其余费!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	if(CheckNum(m_OtherEdit) == FALSE)
	{
		MessageBox("请输入有效其余费!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	//检测用户有没有在图上标记
	int havestore=UNKNOWN,haveclient=UNKNOWN;
	for(int n=51;n<=71;n++)
	{
		switch(node[n].choose)
		{
			case STOREFLAG: havestore=STOREFLAG  ;break;
			case CLIENTFLAG:haveclient=CLIENTFLAG;break;
		}
		if(havestore == STOREFLAG && haveclient == CLIENTFLAG)
			break;
	}

	if(havestore == STOREFLAG && haveclient == CLIENTFLAG)
	{	
		test=TRUE;
		//释放上次头结点指向的内存
		if(plan != -1)
			FreeRoute(Head);
		//图像处理
		ChooseA(atof(m_NeedEdit));
		m_DistanceStr="";
		m_PrimeStr="";
		m_WholeStr="";
		m_SpeedStr="";
		m_TrafficStr="";
		UpdateData(FALSE); //刷新屏幕		
		CString str;
		GetDlgItem(IDC_TestButton)->GetWindowText(str);
		if(str == "测试")
		{
			MessageBox("请您从仓库所在道路开始，双击下次要到的道路。","提示",MB_OK|MB_ICONINFORMATION);
			if(ing == FALSE) //一出来就点测试按钮的情况
			{
				CutScreen();
			}
		}
		PutScreen();

		//处理控件状态
		GetDlgItem(IDC_OkButton)->EnableWindow(FALSE);
		GetDlgItem(IDC_TestButton)->SetWindowText("重测");
		GetDlgItem(IDC_NeedEdit)->EnableWindow(FALSE);
		GetDlgItem(IDC_FuelEdit)->EnableWindow(FALSE);
		GetDlgItem(IDC_OtherEdit)->EnableWindow(FALSE);
		GetDlgItem(IDC_MindisRadio)->EnableWindow(FALSE);
		GetDlgItem(IDC_MintimRadio)->EnableWindow(FALSE);
		GetDlgItem(IDC_StopButton)->EnableWindow(FALSE);
		GetDlgItem(IDC_ReplayButton)->EnableWindow(FALSE);
		GetDlgItem(IDC_ResetButton)->EnableWindow(TRUE);

		//标记处理
		ing=TRUE;

		for(int i=1;i<=71;i++) 
		{
			node[i].running=FALSE;
			node[i].been=FALSE;
		}

		for(int store=51;store<=71;store++) //找仓库的ID
		{
			if(node[store].choose==STOREFLAG)
			{
				lastnode=store; //初始化上次节点
				break;
			}
		}
		
		//画起始仓库处的红点
		CDC *pDC=GetDC();
		pDC->SelectObject(&pen1); //将画笔选入设备描述表		
		pDC->MoveTo((int)node[store].x,(int)node[store].y);
		pDC->LineTo((int)node[store].x,(int)node[store].y);
		pDC->DeleteDC();

		//取出仓库所在的那条道路，并将仓库加入Head首节点
		Head=(ROUTE*)malloc(sizeof(ROUTE));
		Start=(ROUTE*)malloc(sizeof(ROUTE));
		Head->id=0;Head->next=Start;
		Start->id=store;Start->next=NULL;
		Link=Start;

		//初始化左边
		xi=node[Link->id].x;
		yi=node[Link->id].y;

/**************此时Head->next->id存的是store的ID，之后的任务交给双击响应函数来添加***************/
	}
	else
	{
		MessageBox("您还未标记仓库及客户地址!","警告",MB_OK|MB_ICONSTOP);
		return;
	}	
	
}
/***************************************************************************************************************************************/
void CMyDlg::OnHelpMenu() //帮助菜单
{
	//弹出使用说明对话框
	CHelpDlg dlg;
	dlg.DoModal();
}
/***************************************************************************************************************************************/
void CMyDlg::OnLButtonDblClk(UINT nFlags, CPoint point) //左键双击
{
	if(test == TRUE) //测试按钮点后才进行以下处理
	{
		if     ((point.x>=219&&point.x<=225)&&(point.y>=231&&point.y<=271)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(1 )==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=219&&point.x<=225)&&(point.y>=271&&point.y<=333)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(2 )==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=219&&point.x<=225)&&(point.y>=333&&point.y<=416)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(3 )==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=219&&point.x<=225)&&(point.y>=416&&point.y<=494)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(4 )==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=224&&point.x<=306)&&(point.y>=268&&point.y<=274)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(5 )==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=306&&point.x<=338)&&(point.y>=268&&point.y<=283)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(6 )==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=222&&point.x<=338)&&(point.y>=330&&point.y<=336)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(7 )==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=222&&point.x<=338)&&(point.y>=413&&point.y<=419)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(8 )==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=222&&point.x<=340)&&(point.y>=491&&point.y<=497)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(9 )==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=337&&point.x<=343)&&(point.y>=228&&point.y<=280)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(10)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=337&&point.x<=343)&&(point.y>=277&&point.y<=333)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(11)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=337&&point.x<=342)&&(point.y>=333&&point.y<=414)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(12)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=337&&point.x<=343)&&(point.y>=414&&point.y<=494)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(13)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=340&&point.x<=384)&&(point.y>=277&&point.y<=283)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(14)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=340&&point.x<=384)&&(point.y>=330&&point.y<=336)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(15)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=338&&point.x<=426)&&(point.y>=410&&point.y<=418)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(16)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=340&&point.x<=426)&&(point.y>=491&&point.y<=497)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(17)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=381&&point.x<=387)&&(point.y>=280&&point.y<=333)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(18)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=423&&point.x<=429)&&(point.y>=332&&point.y<=413)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(19)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=423&&point.x<=429)&&(point.y>=413&&point.y<=493)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(20)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=384&&point.x<=476)&&(point.y>=277&&point.y<=283)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(21)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=384&&point.x<=426)&&(point.y>=330&&point.y<=336)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(22)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=426&&point.x<=476)&&(point.y>=330&&point.y<=335)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(23)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=426&&point.x<=476)&&(point.y>=410&&point.y<=416)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(24)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=426&&point.x<=476)&&(point.y>=490&&point.y<=496)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(25)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=473&&point.x<=477)&&(point.y>=218&&point.y<=277)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(26)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=473&&point.x<=477)&&(point.y>=277&&point.y<=330)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(27)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=473&&point.x<=479)&&(point.y>=330&&point.y<=413)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(28)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=473&&point.x<=479)&&(point.y>=413&&point.y<=492)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(29)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=476&&point.x<=506)&&(point.y>=215&&point.y<=222)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(30)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=477&&point.x<=493)&&(point.y>=274&&point.y<=288)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(31)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=493&&point.x<=503)&&(point.y>=265&&point.y<=285)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(32)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=477&&point.x<=493)&&(point.y>=285&&point.y<=328)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(33)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=476&&point.x<=505)&&(point.y>=327&&point.y<=333)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(34)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=476&&point.x<=505)&&(point.y>=410&&point.y<=416)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(35)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=476&&point.x<=538)&&(point.y>=489&&point.y<=495)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(36)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=502&&point.x<=508)&&(point.y>=40 &&point.y<=68 )&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(37)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=502&&point.x<=508)&&(point.y>=68 &&point.y<=123)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(38)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=504&&point.x<=508)&&(point.y>=123&&point.y<=218)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(39)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=502&&point.x<=508)&&(point.y>=218&&point.y<=265)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(40)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=502&&point.x<=508)&&(point.y>=265&&point.y<=330)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(41)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=502&&point.x<=508)&&(point.y>=330&&point.y<=413)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(42)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=505&&point.x<=538)&&(point.y>=65 &&point.y<=71 )&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(43)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=505&&point.x<=538)&&(point.y>=120&&point.y<=126)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(44)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=505&&point.x<=538)&&(point.y>=215&&point.y<=221)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(45)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=505&&point.x<=538)&&(point.y>=327&&point.y<=333)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(46)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=505&&point.x<=538)&&(point.y>=410&&point.y<=416)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(47)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=535&&point.x<=541)&&(point.y>=68 &&point.y<=123)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(48)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=535&&point.x<=541)&&(point.y>=123&&point.y<=218)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(49)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=535&&point.x<=539)&&(point.y>=218&&point.y<=286)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(50)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=535&&point.x<=539)&&(point.y>=286&&point.y<=330)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(51)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=535&&point.x<=541)&&(point.y>=330&&point.y<=413)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(52)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=535&&point.x<=541)&&(point.y>=413&&point.y<=491)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(53)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=530&&point.x<=541)&&(point.y>=491&&point.y<=596)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(54)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=538&&point.x<=612)&&(point.y>=327&&point.y<=331)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(55)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=538&&point.x<=584)&&(point.y>=410&&point.y<=416)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(56)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=584&&point.x<=648)&&(point.y>=412&&point.y<=424)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(57)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=541&&point.x<=595)&&(point.y>=491&&point.y<=506)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(58)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=595&&point.x<=642)&&(point.y>=503&&point.y<=511)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(59)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=610&&point.x<=631)&&(point.y>=330&&point.y<=363)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(60)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=631&&point.x<=651)&&(point.y>=363&&point.y<=386)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(61)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=647&&point.x<=653)&&(point.y>=385&&point.y<=424)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(62)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=646&&point.x<=651)&&(point.y>=424&&point.y<=456)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(63)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=645&&point.x<=648)&&(point.y>=456&&point.y<=469)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(64)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=640&&point.x<=647)&&(point.y>=469&&point.y<=508)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(65)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=653&&point.x<=710)&&(point.y>=385&&point.y<=408)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(66)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=650&&point.x<=665)&&(point.y>=422&&point.y<=427)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(67)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=642&&point.x<=678)&&(point.y>=506&&point.y<=515)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(68)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=678&&point.x<=701)&&(point.y>=509&&point.y<=515)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(69)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=701&&point.x<=712)&&(point.y>=515&&point.y<=529)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(70)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=710&&point.x<=723)&&(point.y>=408&&point.y<=432)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(71)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=720&&point.x<=725)&&(point.y>=433&&point.y<=478)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(72)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=711&&point.x<=722)&&(point.y>=478&&point.y<=530)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(73)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=529&&point.x<=538)&&(point.y>=215&&point.y<=221)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(74)==TRUE) {SetTimer(1,speed,NULL);}
		else if((point.x>=508&&point.x<=535)&&(point.y>=223&&point.y<=265)&&xi==node[Link->id].x&&yi==node[Link->id].y&&JoinTable(75)==TRUE) {SetTimer(1,speed,NULL);}
	}
	
	CDialog::OnLButtonDblClk(nFlags, point);
}
/***************************************************************************************************************************************/
/*普通函数定义*/
/***************************************************************************************************************************************/
int* Array() //由用户选择产生顺序数组
{
	//按需申请数组并将选中ID加入数组
	int store,client,num=4,i; //num用来统计数组总数
	for(client=51;client<=71;client++) //client记录的是终点的ID
		if(node[client].choose == CLIENTFLAG)
			num++;
	int *order=(int*)malloc(sizeof(int)*num); //存储结点顺序的数组
	for(store=51;store<=71;store++) //store记录的是仓库的ID
		if(node[store].choose==STOREFLAG)
			break;

	for(i=2,client=51;client<=71;client++) //把客户ID放入数组
		if(node[client].choose==CLIENTFLAG)
			order[i++]=client;
	order[1]=store; //首仓库
	order[num-2]=store; //尾仓库
	order[num-1]=0; //结束标志

	return order;
}
/***************************************************************************************************************************************/
int* Product(int *order,int n) //产生新解(用指针改变数组顺序)
{
	srand((unsigned)time(NULL)); //随机种子
	int *p,tmp;

	int i=0;
	while(*(order+2+i+1) != 0)//i到达最后一个元素了
	{
		p=order+2+(i+rand()%(n-i-4));
		
		tmp=*p;
		*p=*(order+2+i);
		*(order+2+i)=tmp;		
			
		i++;
	}	
	
	//将沿路的顺序优化
	FreeRoute(FindCircle(order));

	return order;
}
/***************************************************************************************************************************************/
ROUTE* FindRoute(int *store,int *client)
{
	node[*store].father=*store; //为首次估价做准备

	ASTAR *head1,*head2,*head3,*start,*inlink,*inlink1,*clink,*tlink; //1用于OPEN表 2用于CLOSE表
	ASTAR *min;
	int   i; //结点周围的id推移
	int  *p; //指向数组的指针并推移
	
	head1=(ASTAR*)malloc(sizeof(ASTAR)); //申请头结点空间(OPEN)
	head2=(ASTAR*)malloc(sizeof(ASTAR)); //申请头结点空间(CLOSE)
	head3=(ASTAR*)malloc(sizeof(ASTAR)); //申请头结点空间(TRAFFIC)
	inlink=head1; //所有表连接用
	clink=head2; //CLOSE表连接用
	tlink=head3; //TRAFFIC表连接用
	start=(ASTAR*)malloc(sizeof(ASTAR)); //开辟OPEN表首结点
	
	//OPEN表初始化
	start->id=*store; //首结点为仓库
	start->center=0;
	start->g=0;
	start->price=fabs(node[*store].x-node[*client].x)+fabs(node[*store].y-node[*client].y); //计算起点f(n)值
	inlink->next=start; //新结点连到表尾
	inlink=start; //移动指针下移
	inlink->next=NULL; //初始化结束标志

	//CLOSE表初始化
	clink->next=NULL; //暂无首结点,添加用

	//TRAFFIC表初始化
	tlink->next=NULL;//添加用

	//将最短路径点排入OPEN表
	while(true) //退出的条件是找到终点
	{
		inlink=head1->next;
		if(inlink == NULL) //由于拥挤道路太多已经扩展完所有结点，找出拥挤节点中h最小的一个给min
		{
			inlink1=head3->next;
			min=inlink1;
			while(inlink1 != NULL)
			{
				if(inlink1->price-inlink1->g < min->price-min->g) //比h谁小
				{
					min=inlink1;
				}
				inlink1=inlink1->next;
			}	
			node[min->id].father=min->center; //指向中心节点
		}
		else
		{
			min=inlink;
		}
			
		while(inlink != NULL) //在OPEN中找f(n)最小的结点（由min指向）作为当前结点
		{
			if((inlink->price) == (min->price) && node[inlink->id].choose == CLIENTFLAG) //沿路经过用户标记的客户
			{
				node[inlink->id].choose=HIDEFLAG; //标记为已配送，产生新路线时又要解除成CLIENTFLAG
				min=inlink;
				break;
			}
			if((inlink->price) < (min->price) || ((inlink->price)==(min->price) && (fabs(node[*client].x-node[inlink->id].x)+fabs(node[*client].y-node[inlink->id].y) < fabs(node[*client].x-node[min->id].x)+fabs(node[*client].y-node[min->id].y))) || min->id == 30 && inlink->id == 49) //比price谁小，若一样比h谁小
				min=inlink;	
			inlink=inlink->next; //下移
		}

		if((min->id) == *client) //扩展到目标客户
		{
			start=(ASTAR*)malloc(sizeof(ASTAR)); //将最后的一个结点也加入CLOSE表
			start->id=min->id;
			start->center=0;
			start->g=min->g;
			clink->next=start;
			clink=start;
			clink->next=NULL;

			break;
		}

		//检测*store与*client两地点之间是否要沿途经过数组里面的其他客户
		p=client; //指针指向client的id
		while(*(p+1) != 0) //找到具体p位置
		{
			if(min->id == *p)//将沿路客户提到前面来，
			{
				if(*(client+1) == *p)//特殊情况 沿路客户恰在client之后一个
				{
					int tmp;
					tmp=*client;*client=*p;*p=tmp;
				}
				else//一般情况
				{
					int tmp;
					tmp=*client;*client=*(client+1);*(client+1)=tmp;
					tmp=*client;*client=*p;*p=tmp;
				}
			
				start=(ASTAR*)malloc(sizeof(ASTAR));//将最后的一个结点也加入CLOSE表
				start->id=min->id;
				start->center=0;
				start->g=min->g;
				clink->next=start;
				clink=start;
				clink->next=NULL;
				
				goto ADJUST;
			}
			p++;//指针向下推移
		}

	    i=0; //每次都是从相邻结点中的第一个开始搜索
NEXT:	while((node[min->id].aroundid[i]) != 0) //遍及当前结点的所有相邻结点
		{
			crowd=CheckTraffic(min->id,node[min->id].aroundid[i]); //中心结点与其相邻结点所连道路是否拥挤
			
			//成立则加入TRAFFIC表
			//不过首先查找周围点是否已经在CLOSE表中,排除把先前那些测过的点加进去
			inlink=head2;
			while(inlink != NULL)
			{
				if((inlink->id) == (node[min->id].aroundid[i]))
					break;
				inlink=inlink->next; //下移
			}
			if(inlink == NULL && crowd == FALSE) //没在CLOSE表中，则把node[min->id].aroundid[i]添加进TRAFFIC表
			{
				inlink1=head3->next; //赋予TRAFFIC表头
				while(inlink1 != NULL) //查找该相邻结点是否已经在TRAFFIC表中了
				{
					if(inlink1->id == node[min->id].aroundid[i] && inlink1->center == min->id) //相同结点
						break;
					inlink1=inlink1->next;
				}
				if(inlink1 == NULL) //新来的拥挤结点
				{
					start=(ASTAR*)malloc(sizeof(ASTAR));
					start->id=node[min->id].aroundid[i];
					start->center=min->id;
					start->g=(min->g)+sqrt(pow(node[min->id].x-node[node[min->id].aroundid[i]].x,2)+pow(node[min->id].y-node[node[min->id].aroundid[i]].y,2));//初次添加g(n)
					start->price=(start->g)+fabs(node[node[min->id].aroundid[i]].x-node[*client].x)+fabs(node[node[min->id].aroundid[i]].y-node[*client].y);//初次添加f(n)
					tlink->next=start;
					tlink=start;
					tlink->next=NULL;
				}		
			}

			//为相邻结点选择他们的父结点的前提条件
			if(  plan == 0 //最短路程
			  ||(plan == 1 && crowd) //最短时间此路可走
			  ||(CheckSurvive(min->id)) //父结点无路可走
			  ||(node[min->id].aroundid[i] == *client) //终点近在眼前
			  ||(CheckSurvive(node[min->id].aroundid[i]))) //子点无路可走
			{
				//查找ID为min->node的中心结点的相邻结点是否已经在OPEN表中
				inlink=head1->next; //赋予OPEN表头
				while(inlink != NULL)
				{
					double newprice=min->g+sqrt(pow(node[min->id].x-node[node[min->id].aroundid[i]].x,2)+pow(node[min->id].y-node[node[min->id].aroundid[i]].y,2))+fabs(node[node[min->id].aroundid[i]].x-node[*client].x)+fabs(node[node[min->id].aroundid[i]].y-node[*client].y); //计算新的price
					if((inlink->id) == (node[min->id].aroundid[i])) //相邻结点确实已经在OPEN里了	
					{
						if((newprice) < (inlink->price)) //如果比以前那个price小则更新并修改其父结点
						{
							node[node[min->id].aroundid[i]].father=(min->id);//将当前中心结点设置为该相邻结点的父结点
							inlink->g=(min->g)+sqrt(pow(node[min->id].x-node[node[min->id].aroundid[i]].x,2)+pow(node[min->id].y-node[node[min->id].aroundid[i]].y,2));//修改g(n)
							inlink->price=newprice;//修改f(n)
						}
						i++;
						goto NEXT; //查找下个相邻结点
					}
					inlink=inlink->next;	
				}	

				//查找是否已经在CLOSE表中（曾经已是中心结点）
				inlink=head2;
				while(inlink != NULL)
				{
					if((inlink->id) == (node[min->id].aroundid[i]))
					{
						i++;
						goto NEXT;
					}
					inlink=inlink->next;
				}
				
				//能执行到这一步说明是既没在OPEN中也没在CLOSE中的游离结点
				node[node[min->id].aroundid[i]].father=(min->id); //指向该父节点			
			
				start=(ASTAR*)malloc(sizeof(ASTAR)); //此结点已经打开那么加入OPEN表中
				start->id=node[min->id].aroundid[i];
				start->center=0;
				start->g=(min->g)+sqrt(pow(node[min->id].x-node[node[min->id].aroundid[i]].x,2)+pow(node[min->id].y-node[node[min->id].aroundid[i]].y,2));//初次添加g(n)
				start->price=(start->g)+fabs(node[node[min->id].aroundid[i]].x-node[*client].x)+fabs(node[node[min->id].aroundid[i]].y-node[*client].y);//初次添加f(n)
				
				inlink=head1;//找OPEN表表尾并把新结点加入
				while(inlink->next != NULL)
					inlink=inlink->next;
				inlink->next=start;
				inlink=start;
				inlink->next=NULL;
			}
			i++; //下一个结点
		}

		//当前结点考察完毕，加入CLOSE表
		start=(ASTAR*)malloc(sizeof(ASTAR));		
		start->id=min->id;start->center=0;start->g=min->g;
		clink->next=start;
		clink=start;
		clink->next=NULL;
	
		//并将其从OPEN表中删除
		if(min->center == 0) //不是复活点才可以删除
		{
			inlink1=head1;inlink=head1->next;
			while(inlink->id != min->id)
			{
				inlink1=inlink;
				inlink=inlink->next;
			}
			inlink1->next=inlink->next;
			free(inlink);inlink=NULL;
		}
	}

ADJUST:
	//释放OPEN表
	FreeAstar(head1);

	//寻找CLOSE表中最后一个元素取其g(n)路程值以后给Head->id
	inlink=head2;
	while(inlink->next != NULL)
		inlink=inlink->next;

	//建立溯回路线链表	
	ROUTE  *head; //最后路线链表头结点
	head=(ROUTE*)malloc(sizeof(ROUTE));
	link=head;
	link->id=(int)inlink->g;link->next=NULL;

	//释放CLOSE表
	FreeAstar(head2);
	//释放TRAFFIC表
	FreeAstar(head3);

	node[*store].father=UNKNOWN;
	Goback(*client); //寻找路线、装入链表（此时只要知道client就一定能追溯其最短路线）
	
	return head;
}
/***************************************************************************************************************************************/
ROUTE* FindCircle(int *order) //产生巡回链表
{
	//用于接收产生的链表
	ROUTE *head;
	head=(ROUTE*)malloc(sizeof(ROUTE)); //新巡回链表头指针
	Start=(ROUTE*)malloc(sizeof(ROUTE));
	head->id=0;head->next=Start;
	Start->id=*(order+1);Start->next=NULL;
	lin=Start;

	for(int i=1;*(order+i+1)!=0;i++) //按数组中的顺序遍历以求总路程、总路径
	{		
		//找表尾
		while(lin->next!=NULL)
			lin=lin->next;
			
		//累加路线长度	
		lin->next=FindRoute(order+i,order+i+1); //依次取前、次结点找两点距离
		head->id+=(lin->next)->id; //把产生的路线长度累加到最终头结点中

		//连接新链表
		lin->next=((lin->next)->next)->next;	

		//清空各个结点的父结点信息
		for(int j=1;j<=71;j++)
			node[j].father=UNKNOWN;
	}

	//解除刚才暂时隐藏的客户标记
	for(i=51;i<=71;i++)
		if(node[i].choose==HIDEFLAG)
			node[i].choose=CLIENTFLAG;

	return head;
}
/***************************************************************************************************************************************/
ROUTE* Cool(int *order,int num) //退火算法求最终巡回链表
{	
	double    r;   //r:用于控制降温的快慢
	double    T;   //T:系统温度,系统初始应该要处于一个高温的状态 
	double    T_min; //T_min:温度的下限，若温度T达到T_min，则停止搜索
	double    dE,de;    //dE:价值差 de:变异差 
		
	int *order1=(int*)malloc(sizeof(int)*num);//变异后的新数组
	for(int i=0;i<num;i++)
		order1[i]=order[i];

	
	ROUTE    *old,*fresh,*remember; //old:当前链表、fresh:新解链表（有时变异）、remember:记忆链表
	int      stand=0; //允许一个父亲数组变异的最大次数
	int      times=0; //同文件中历史最小比较大小的最大次数
	
	//振荡初始数组10次
	if(num > 4) 
	{
		for(i=0;i<10;i++)
			order=Product(order,num);
	}

	//三者初始化
	remember=old=FindCircle(order);
	if(mode == 1)	    remove("mindistance.txt"); //非收敛重找模式
	old=Recool(old,1); //初次写入文件最优解
	if(num > 4) order=Product(order,num); //为第一次fresh准备


	r=0.998;
	T=5000;
	T_min=10;
	stand=0;

	while(T > T_min) //未达最低温度就继续搜索
	{
		fresh=FindCircle(order); //产生新链表

		dE=(double)((fresh->id)-(old->id)); //新解与旧解比长度
		de=(double)((fresh->id)-(remember->id)); //新解与记忆解比长度

		if(dE < 0 && de < 0)
			fresh=Recool(fresh,1); //fresh和文件最优解比较，小则覆盖

		if(stand <= 50)//50次变异机会
		{
			//fresh与remember比较
			if(de > 0)//比记忆的大
			{
				stand++; //说明有恶性趋势则少一次变异机会
			}
			else//比记忆的小
			{
				stand--; //说明有优良趋势多给一次变异机会
			}

			//fresh与old比较
			if (dE <= 0) //新解更小
			{
				if(old != NULL && old != remember && old != fresh)
					FreeRoute(old); //舍弃旧解
				old=fresh; //接受新解

				//打乱旧数组，产生新数组
				if(num > 4) order=Product(order,num);
			}
			else //新解大了点
			{
				if(exp(-dE/T) > (double)(rand()%101)/100 && stand == 0) //exp(-dE/T)的概率接受从fresh
				{
					//对此时的old与曾经记忆的remember比较
					if(remember->id > old->id)
					{
						if(remember != NULL && remember != fresh && remember != old)
							FreeRoute(remember); //舍弃记忆
						remember=old;//赋予新记忆链表
						for(int i=0;i<num;i++)//赋予新记忆的数组顺序
							order1[i]=order[i];	
					}
					old=fresh;//勉强接受新解
				}
				
				if(num > 4) order=Product(order,num);
			}
		}
		else //终止恶性变异
		{
			stand=0;//重新给予下个父亲数组5次变异机会
			if(old != NULL && old != fresh && old != remember)
				FreeRoute(old); //舍弃记忆
			old=remember; //把记忆链表给旧链表
			//新解由记忆里的order1产生
			if(num>4) order=Product(order1,num);
		}

		//释放fresh
		if(fresh != NULL && fresh != old && fresh != remember)
			FreeRoute(fresh); //舍弃记忆
		
		T=r*T;  //降温退火
	}

	free(order1);order1=NULL; //释放记忆数组
	if(remember != NULL && remember != old) FreeRoute(remember);
	if(adjust == TRUE) remove("adjust.txt");
	
	old=Recool(old,5); //old吸取文件最优解

	finalArray=order;
	return old;
}
/***************************************************************************************************************************************/
ROUTE* Recool(ROUTE *old,int times) //重升温
{
	int  record; //读记录值
	FILE *rp,*wp; //读、写文件指针

	if(adjust == FALSE)	rp=fopen("mindistance.txt","r");
	else				rp=fopen("adjust.txt","r");

	if(rp == NULL) //没有文件存在，写入整个链表
	{
		if(adjust == FALSE)	wp=fopen("mindistance.txt","w");
		else				wp=fopen("adjust.txt","w"); //以写的方式打开
		
		link=old; //写入整个链表
		while(link != NULL)
		{
			fprintf(wp,"%d ",link->id);
			link=link->next;
		}

		fclose(wp);
		return old;
	}
	else
	{
		fscanf(rp,"%d ",&record); //读入旧记录
		fclose(rp);
		if(old->id <= record) //新记录
		{
			if(adjust == FALSE)	wp=fopen("mindistance.txt","w");
			else				wp=fopen("adjust.txt","w");
			
			link=old; //写入整个链表
			while(link != NULL)
			{
				fprintf(wp,"%d ",link->id);
				link=link->next;
			}

			fclose(wp);
			return old;
		}
		else //大于
		{
			if(times == 5) //达到最大查找次数
			{
				if(adjust == FALSE)	rp=fopen("mindistance.txt","r");
				else				rp=fopen("adjust.txt","r");

				fscanf(rp,"%d ",&record);
				ROUTE *history=(ROUTE*)malloc(sizeof(ROUTE));
				history->id=record;
				history->next=NULL;

				link=history;
				while(!feof(rp)) //一直读到文件尾部
				{
					fscanf(rp,"%d ",&record);
				
					Start=(ROUTE*)malloc(sizeof(ROUTE));
					Start->id=record;
					Start->next=NULL;
					link->next=Start;
					link=Start;
				}
				fclose(rp);

				FreeRoute(old);
				return history;
			}
			else return old;
		}
	}
}
/***************************************************************************************************************************************/
BOOL CheckBlank(CString str) //检测填入信息是否为空
{
	if(str == "")
		return FALSE;
	else
		return TRUE;
}
/***************************************************************************************************************************************/
BOOL CheckNum(CString str) //检测填入数字是否非法(只能是数字)
{
	BOOL firstnum=FALSE,firstpoint=FALSE; //分别表示个位数字未输入、第一个小数点未输入
	
	if(atof(str) <= 0)
	{
		return FALSE;
	}

	for(int i=0;i<str.GetLength();i++)
	{
		if(str.GetAt(i) >= '0' && str.GetAt(i) <= '9') //输入的是数字
		{
			firstnum=TRUE;
			continue;
		}
		else if(str.GetAt(i) == '.' && firstnum == TRUE && firstpoint == FALSE) //正确小数形式(.只能输入一次且个位必须有数)
		{
			firstpoint=TRUE;
			continue;
		}
		else //其余情况均非法
			return FALSE;
	}
	
	return TRUE;
}
/***************************************************************************************************************************************/
BOOL CheckSurvive(int one) //检查是否无路可走
{
	if(one >= 51 && one <= 71) //是客户结点
	{
		return FALSE;
	}
	else
	{
		int i=0;
		//首先检查是不是无路可走的情况
		while(node[one].roadid[i]!=0)
		{
			int k=node[one].roadid[i];
			if(road[k].traffic == SMOOTH) //还是有一条生路可走就马上退出，此时没有到结束标志
				break;
			i++;
		}
		if(node[one].roadid[i] == 0) //所有道路都已查完，还是未能找到流畅的
		{
			return TRUE; //允许加入该结点
		}
		else
		{
			return FALSE;
		}
	}
}
/***************************************************************************************************************************************/
BOOL CheckTraffic(int one,int two) //检查两个结点共有道路是否拥堵
{	
	//由已经确定的两结点查出是哪条路被占用了
	for(int i=0;node[one].roadid[i]!=0;i++) //查first周围哪条路与second相同
	{
		int r=node[one].roadid[i]; //赋予周围道路id,加快速度用
		for(int j=0;node[two].roadid[j]!=0;j++) //查second周围哪条路与first相同
		{	
			int m=node[two].roadid[j];
			if(r == m) //确认one和two含有相同的道路
			{
				if(road[r].traffic == SMOOTH) //流畅
					return TRUE;
				else //忙碌
					return FALSE;
			}	
		}
	}

	return TRUE;
}
/***************************************************************************************************************************************/
BOOL CheckNode(int first,int second) //查找first点周围是否存在second点
{
	for(int j=0;node[first].aroundid[j]!=0;j++)
		if(node[first].aroundid[j] == second) //周围确实有second点
			return TRUE;

	return FALSE;
}
/***************************************************************************************************************************************/
void ChangeCircle() //动态规划新路线
{
	//找旧表的下一个客户是谁，由links指向
	ROUTE *links;
	int   nextplace=0;

	//找剩余路线中哪个客户点离当前所在结点最近
	if(node[Link->id].choose != CLIENTFLAG) //排除下一个结点就是客户结点的情况
	{
		links=Link;
		while(links->next != NULL)
		{
			if(links->id >= 51 && links->id <= 71 && node[links->id].choose != STOREFLAG) //客户点
			{
				nextplace=links->id;
				break;
			}
			links=links->next;
		}
		if(nextplace == 0) //只剩最后一个仓库的情况
		{
			nextplace=links->id;
		}
	}
	else
	{
		nextplace=Link->id;
	}
			
	//打开方才暂时隐藏的客户标记
	for(int i=51;i<=71;i++)
		if(node[i].choose == HIDEFLAG)
			node[i].choose=CLIENTFLAG;	
		
/************************************/
/*由没有配送到的客户建立新数组*/
/************************************/
		
	//申请内存
	int num=3;
	for(i=51;i<=71;i++)
		if(node[i].choose == CLIENTFLAG && node[i].been == FALSE)
			num++;
	int *order;
	order=(int*)malloc(sizeof(int)*num);
	
	//将Head里面没有配送的客户重新用order存放
	i=2;
	links=Link;
	while(links->next != NULL)
	{
		if(links->id != nextplace && links->id >= 51 && links->id <= 71)
		{
			*(order+i)=links->id;
			i++;
		}
		links=links->next;
	}
	*(order+1)=nextplace;
	*(order+num-2)=links->id;
	*(order+num-1)=0;

//此时以order为首地址的数组存储的就是Head->id路线里面的剩余的顺序

	//查找含有minplace的第一个指针
	links=Link;
	while(links->id != nextplace)
		links=links->next;
	
	if(CheckCool(links) == TRUE) //需要对剩余客户重新退火
	{
		//连接路线求新的Head
		if(Link->id < 51) //若下一次是普通结点，则求普通结点---下一个结点的距离
		{
			FreeRoute(Link->next);
			Link->next=FindRoute(&(Link->id),&(nextplace))->next->next;
		}
		//找新表的minplace
		links=Link;
		while(links->id != nextplace)
			links=links->next;
		links->next=Cool(order,num)->next->next; //再次套用退化算法求小巡回
	}
	else
	{
		if(Link->id < 51) //若下一次是普通结点，则求普通结点---下一个结点的距离
		{
			Link->next=FindRoute(&(Link->id),&(nextplace))->next->next;
		}

		ROUTE *front=Link;
		while(front->next != NULL)
			front=front->next;

		front->next=links->next;
	}
}
/***************************************************************************************************************************************/
void Goback(int son) //回溯路线、装入链表
{
	if(son != UNKNOWN)//未到store的父结点UNKNOWN
	{
		Goback(node[son].father);//递归下去
		
		Start=(ROUTE*)malloc(sizeof(ROUTE));//动态创建新结点
		Start->id=son;
		link->next=Start;
		link=Start;
		link->next=NULL;
	}
	else
	{
		return;
	}
}
/***************************************************************************************************************************************/
void FreeAstar(ASTAR *h) //释放ASTAR链表
{
	ASTAR *inlink=h;
	while(inlink != NULL)
	{
		h=inlink;
		inlink=inlink->next;
		free(h);
		h=NULL;
	}
}
/***************************************************************************************************************************************/
void  FreeRoute(ROUTE *h) //释放ROUTE链表
{
	ROUTE *inlink=h;
	while(inlink != NULL)
	{
		h=inlink;
		inlink=inlink->next;
		free(h);
		h=NULL;
	}
}
/***************************************************************************************************************************************/
void AddRunning(int first,int second) //道路占用添加
{
	//由已经确定的两结点查出是哪条路被占用了
	for(int i=0;node[first].roadid[i]!=0;i++) //查first周围哪条路与second相同
	{
		int r=node[first].roadid[i]; //赋予周围道路id,加快速度用
		for(int j=0;node[second].roadid[j]!=0;j++) //查second周围哪条路与first相同
		{	
			if(r == node[second].roadid[j]) //确认first和second占用的道路一样
				road[r].running=TRUE;
		}
	}
}
/***************************************************************************************************************************************/
void ChooseA(double need) //确定常量
{
	//筛选货车额定功率
	if	   (need > 0 && need <= 4 ) {P=120; O=0.08; M=2+need;FprintfA(2.0);}
	else if(need > 4 && need <= 8 ) {P=160; O=0.12; M=5+need;FprintfA(5.0);}
	else if(need > 8 && need <= 12) {P=200; O=0.16; M=8+need;FprintfA(8.0);}	
}
/***************************************************************************************************************************************/
void FprintfA(double G) //常量写入文件
{
	FILE *wp=fopen("CarA.txt","w");	
	fprintf(wp,"%lf %lf %lf ",P,G,O);
	fclose(wp);
}
/***************************************************************************************************************************************/
void CalculateTime() //计算时间
{
	T=0;
	ROUTE *s=Head->next;
	while(s->next != NULL)
	{
		T+=(CalculateDis(s->id,s->next->id)/30)/CalculateSpeed(s->id,s->next->id);
		s=s->next;
	}
}
/***************************************************************************************************************************************/
void CalculateHead() //计算Head链表总路程
{
	Head->id=0;
	ROUTE *l=Head->next;		
	while(l->next != NULL)
	{
		Head->id+=(int)CalculateDis(l->id,l->next->id); //累加路程
		l=l->next;
	}
}
/***************************************************************************************************************************************/
double CalculateDis(int first,int second) //计算两节点距离
{
	return sqrt(pow(node[first].x-node[second].x,2)+pow(node[first].y-node[second].y,2));
}
/***************************************************************************************************************************************/
double CalculateSpeed(int first,int second) //计算速度
{
	if(CheckTraffic(first,second) == FALSE) //所在道路拥堵
	{
		return 3.6+(double)(rand()%6)/10; //速度为3.6km/h左右
	}
	else //流畅
	{
		return P/M*3.6-7+rand()%15;	
	}
}
/***************************************************************************************************************************************/
BOOL CMyDlg::JoinTable(int r) //检测传入路段r是否可同上次路段衔接,若能则加入Head链表
{
	int first,second;
	
	//找有r路段的节点是哪两个
	//优先找客户结点
	for(int i=51;i<=71;i++)
	{
		for(int j=0;node[i].roadid[j]!=0;j++)
		{
			if(node[i].roadid[j] == r && CheckNode(i,lastnode) == TRUE && (node[i].choose == CLIENTFLAG || node[i].choose == STOREFLAG)) //预示该节点含有形参道路,并且它含有上次到达的节点
			{
				lastnode=i; //更换上次节点

				//将i这个节点连接到Head链表中
				Start=(ROUTE*)malloc(sizeof(ROUTE));
				Start->id=i;Start->next=NULL;
				first=Link->id;
				Link->next=Start;
				Link=Start;second=Link->id;
				sonx=node[second].x;
				sony=node[second].y;

				//文本信息反映
				CalculateHead();
				ChooseA(atof(m_NeedEdit));
				V=CalculateSpeed(first,second);
				Statistic();

				return TRUE;
			}
		}
	}

	//其后找普通结点
	for(i=1;i<=49;i++)
	{
		for(int j=0;node[i].roadid[j]!=0;j++)
		{
			if(node[i].roadid[j] == r && CheckNode(i,lastnode) == TRUE) //预示该节点含有形参道路,并且它含有上次到达的节点
			{
				lastnode=i; //更换上次节点

				//将i这个节点连接到Head链表中
				Start=(ROUTE*)malloc(sizeof(ROUTE));
				Start->id=i;Start->next=NULL;
				first=Link->id;
				Link->next=Start;
				Link=Start;second=Link->id;
				sonx=node[second].x;
				sony=node[second].y;
					
				//文本信息反映
				CalculateHead();
				ChooseA(atof(m_NeedEdit));
				V=CalculateSpeed(first,second);
				Statistic();

				return TRUE;
			}
		}
	}

	return FALSE; //此道路不可加
}
/***************************************************************************************************************************************/
void CMyDlg::CutScreen() //截图
{
	memDC1.BitBlt(0,0,rt1.Width(),rt1.Height(),pDC1,0,0,SRCCOPY);
}
/***************************************************************************************************************************************/
void CMyDlg::PutScreen() //显示截图
{
	pDC1->BitBlt(179,11,700,700,&memDC1,179,11,SRCCOPY);
}
/***************************************************************************************************************************************/
void CMyDlg::ChangeAnytime(int first,int second) //速度与路况随时更新
{
	V=CalculateSpeed(first,second); //更新V
	m_SpeedStr.Format("%.2lf",V); //随机变一次速度\路况
	if(V >= 3.1 && V <= 4.1)  {m_TrafficStr="拥挤";speed=60;SetTimer(1,60,NULL);} //根据速度改变路况
	else                      {m_TrafficStr="流畅";speed=15;SetTimer(1,5,NULL);}
	UpdateData(FALSE);
}
/***************************************************************************************************************************************/
void CMyDlg::DrawLine() //画行进路线
{
	//画推移点
	CDC *pDC=GetDC();
	pDC->SelectObject(&pen1); //将画笔选入设备描述表		
	//画起始点
	pDC->MoveTo((int)xi,(int)yi);
	//计算坐标
	if     (xi < sonx)	yi=((yi-sony)/(xi-sonx))*((++xi)-sonx)+sony;
	else if(xi > sonx)	yi=((yi-sony)/(xi-sonx))*((--xi)-sonx)+sony;
	else //斜率不存在情况
	{
		if     (yi < sony)  	yi++;
		else if(yi > sony)	    yi--;
	}
	//画推移点
	pDC->LineTo((int)xi,(int)yi);
	pDC->DeleteDC();
}
/***************************************************************************************************************************************/
void CMyDlg::Map() //地图图样
{
	CDC memDC,*pDC=GetDC();
	memDC.CreateCompatibleDC(pDC);
	CBitmap bmp;
	bmp.LoadBitmap(IDB_Map);
	memDC.SelectObject(&bmp);
	pDC->BitBlt(179,11,600,600,&memDC,0,0,SRCCOPY);
	memDC.DeleteDC();
	pDC->DeleteDC();
	bmp.DeleteObject();
}
/***************************************************************************************************************************************/
void CMyDlg::TrafficFlag(int m) //把之前标记了拥堵的道路都重画
{
	if(road[m].traffic == CROWD && ((timer == FALSE && road[m].then == FALSE) || timer == TRUE))
	{
		switch(m)
		{
			case 1 :OnPicture(219,231,225,271);break;
			case 2 :OnPicture(219,271,225,333);break;
			case 3 :OnPicture(219,333,225,416);break;
			case 4 :OnPicture(219,416,225,494);break;
			case 5 :OnPicture(224,268,306,274);break;
			case 6 :OnPicture(306,268,338,283);break;
			case 7 :OnPicture(222,330,338,336);break;
			case 8 :OnPicture(222,413,338,419);break;
			case 9 :OnPicture(222,491,340,497);break;
			case 10:OnPicture(337,228,343,280);break;
			case 11:OnPicture(337,277,343,333);break;
			case 12:OnPicture(337,333,343,414);break;
			case 13:OnPicture(337,414,343,494);break;
			case 14:OnPicture(340,277,384,283);break;
			case 15:OnPicture(340,330,384,336);break;
			case 16:OnPicture(338,410,426,418);break;
			case 17:OnPicture(340,491,426,497);break;
			case 18:OnPicture(381,280,387,333);break;
			case 19:OnPicture(423,332,429,413);break;
			case 20:OnPicture(423,413,429,493);break;
			case 21:OnPicture(384,277,476,282);break;
			case 22:OnPicture(384,330,426,336);break;
			case 23:OnPicture(426,330,476,335);break;
			case 24:OnPicture(426,410,476,416);break;
			case 25:OnPicture(426,490,476,496);break;
			case 26:OnPicture(473,218,477,277);break;
			case 27:OnPicture(473,277,477,330);break;
			case 28:OnPicture(473,330,479,413);break;
			case 29:OnPicture(473,413,479,492);break;
			case 30:OnPicture(476,215,506,222);break;
			case 31:OnPicture(476,274,493,288);break;
			case 32:OnPicture(493,265,503,285);break;
			case 33:OnPicture(477,285,493,328);break;
			case 34:OnPicture(476,327,505,333);break;
			case 35:OnPicture(476,410,505,416);break;
			case 36:OnPicture(476,489,538,495);break;
			case 37:OnPicture(502,40,508,68  );break;
			case 38:OnPicture(502,68,508,123 );break;
			case 39:OnPicture(504,123,508,218);break;
			case 40:OnPicture(502,218,508,265);break;
			case 41:OnPicture(502,265,508,330);break;
			case 42:OnPicture(502,330,508,413);break;
			case 43:OnPicture(505,65,538,71  );break;
			case 44:OnPicture(505,120,538,126);break;
			case 45:OnPicture(505,215,538,221);break;
			case 46:OnPicture(505,327,538,333);break;
			case 47:OnPicture(505,410,538,416);break;
			case 48:OnPicture(535,68,541,123 );break;
			case 49:OnPicture(535,123,541,218);break;
			case 50:OnPicture(535,218,539,286);break;
			case 51:OnPicture(535,286,539,330);break;
			case 52:OnPicture(535,330,541,413);break;
			case 53:OnPicture(535,413,541,491);break;
			case 54:OnPicture(535,491,541,596);break;
			case 55:OnPicture(538,327,612,331);break;
			case 56:OnPicture(538,410,584,416);break;
			case 57:OnPicture(584,412,648,424);break;
			case 58:OnPicture(541,491,595,506);break;
			case 59:OnPicture(595,503,642,511);break;
			case 60:OnPicture(610,330,631,363);break;
			case 61:OnPicture(631,363,651,386);break;
			case 62:OnPicture(647,385,653,424);break;
			case 63:OnPicture(646,424,651,456);break;
			case 64:OnPicture(645,456,651,469);break;
			case 65:OnPicture(640,469,647,508);break;
			case 66:OnPicture(653,385,710,408);break;
			case 67:OnPicture(650,422,665,427);break;
			case 68:OnPicture(642,506,678,515);break;
			case 69:OnPicture(678,509,701,515);break;
			case 70:OnPicture(698,515,712,529);break;
			case 71:OnPicture(710,408,723,432);break;
			case 72:OnPicture(720,433,725,478);break;
			case 73:OnPicture(711,478,722,530);break;
			case 74:OnPicture(529,215,538,221);break;
			case 75:OnPicture(508,223,535,265);break;
		}
	}
	
}
/***************************************************************************************************************************************/
void CMyDlg::StoreFlag() //仓库旗图样
{
	CDC memDC,*pDC=GetDC(); //创建设备上下文对象
	memDC.CreateCompatibleDC(pDC); //动态创建位图控件
	CBitmap bmp; //创建位图对象
	bmp.LoadBitmap(IDB_StoreFlag); //加载位图地址
	memDC.SelectObject(&bmp); //将位图选入设备描述表
	TransparentBlt(pDC->m_hDC,Point2.x-6,Point2.y-29,28,29,memDC,0,0,28,29,RGB(255,255,255)); //透明显示位图在指定位置	
	memDC.DeleteDC(); //删除设备上下文对象
	pDC->DeleteDC(); //删除设备上下文指针
	bmp.DeleteObject(); //删除位图对象
}
/***************************************************************************************************************************************/
void CMyDlg::ClientFlag() //客户旗图样
{
	CDC memDC,*pDC=GetDC();
	memDC.CreateCompatibleDC(pDC);
	CBitmap bmp;
	bmp.LoadBitmap(IDB_ClientFlag);
	memDC.SelectObject(&bmp);
	TransparentBlt(pDC->m_hDC,Point2.x-6,Point2.y-29,28,29,memDC,0,0,28,29,RGB(255,255,255));	
	memDC.DeleteDC();
	pDC->DeleteDC();
	bmp.DeleteObject();
}
/***************************************************************************************************************************************/
void CMyDlg::BeenFlag() //红勾图样
{
	CDC memDC,*pDC=GetDC();
	memDC.CreateCompatibleDC(pDC);
	CBitmap bmp;
	bmp.LoadBitmap(IDB_BeenFlag);
	memDC.SelectObject(&bmp);
	TransparentBlt(pDC->m_hDC,Point2.x-6,Point2.y-29,28,29,memDC,0,0,28,29,RGB(255,255,255));	
	memDC.DeleteDC();
	pDC->DeleteDC();
	bmp.DeleteObject();
}
/***************************************************************************************************************************************/
void CMyDlg::ShowBeen() //筛选勾
{
	for(int i=51;i<=71;i++) //把51-71所有到过的客户打勾
	{
		if(node[i].been == TRUE)
		{
			switch(i)
			{
				case 51:Point2.x=232;Point2.y=231;BeenFlag();break;
				case 52:Point2.x=232;Point2.y=302;BeenFlag();break;
				case 53:Point2.x=232;Point2.y=382;BeenFlag();break;
				case 54:Point2.x=238;Point2.y=463;BeenFlag();break;
				case 55:Point2.x=320;Point2.y=228;BeenFlag();break;
				case 56:Point2.x=320;Point2.y=465;BeenFlag();break;
				case 57:Point2.x=455;Point2.y=378;BeenFlag();break;
				case 58:Point2.x=515;Point2.y=40 ;BeenFlag();break;
				case 59:Point2.x=515;Point2.y=104;BeenFlag();break;
				case 60:Point2.x=518;Point2.y=208;BeenFlag();break;
				case 61:Point2.x=515;Point2.y=295;BeenFlag();break;
				case 62:Point2.x=485;Point2.y=459;BeenFlag();break;
				case 63:Point2.x=546;Point2.y=165;BeenFlag();break;
				case 64:Point2.x=553;Point2.y=364;BeenFlag();break;
				case 65:Point2.x=546;Point2.y=395;BeenFlag();break;
				case 66:Point2.x=558;Point2.y=447;BeenFlag();break;
				case 67:Point2.x=546;Point2.y=475;BeenFlag();break;
				case 68:Point2.x=515;Point2.y=596;BeenFlag();break;
				case 69:Point2.x=680;Point2.y=432;BeenFlag();break;
				case 70:Point2.x=607;Point2.y=538;BeenFlag();break;
				case 71:Point2.x=733;Point2.y=453;BeenFlag();break;
			}
		}
	}
}
/***************************************************************************************************************************************/
void CMyDlg::DarkBlue(int x,int y) //画深蓝点
{
	CDC *pDC=GetDC();
	pDC->SelectObject(&pen2);
	pDC->MoveTo(x,y);
	pDC->LineTo(x,y);	
	pDC->DeleteDC();
}
/***************************************************************************************************************************************/
void CMyDlg::LightBlue(int x,int y) //画浅蓝点
{
	CDC *pDC=GetDC();
	pDC->SelectObject(&pen3);
	pDC->MoveTo(x,y);
	pDC->LineTo(x,y);
	pDC->DeleteDC();
}
/***************************************************************************************************************************************/
void CMyDlg::AllBlue() //画所有蓝点
{
	DarkBlue(232,231);DarkBlue(232,302);DarkBlue(232,382);DarkBlue(238,463);DarkBlue(320,228);
	DarkBlue(322,465);DarkBlue(515,40) ;DarkBlue(515,104);DarkBlue(546,165);DarkBlue(518,208);
	DarkBlue(515,295);DarkBlue(455,378);DarkBlue(485,459);DarkBlue(553,364);DarkBlue(546,395);
	DarkBlue(548,437);DarkBlue(546,475);DarkBlue(515,596);DarkBlue(630,515);DarkBlue(665,417);DarkBlue(733,453);
}
/***************************************************************************************************************************************/
void CMyDlg::OnPicture(int x1,int y1,int x2,int y2) //道路选中
{
	CDC memDC,*pDC=GetDC();
	memDC.CreateCompatibleDC(pDC);
	CBitmap bmp;
	bmp.LoadBitmap(IDB_OnMap);
	memDC.SelectObject(&bmp);
	pDC->BitBlt(x1,y1,x2-x1,y2-y1,&memDC,x1-179,y1-11,SRCCOPY);
	memDC.DeleteDC();
	pDC->DeleteDC();
	bmp.DeleteObject();
}
/***************************************************************************************************************************************/
void CMyDlg::Picture(int x1,int y1,int x2,int y2) //道路重绘
{
	CDC memDC,*pDC=GetDC();
	memDC.CreateCompatibleDC(pDC);
	CBitmap bmp;
	bmp.LoadBitmap(IDB_Map);
	memDC.SelectObject(&bmp);
	pDC->BitBlt(x1,y1,x2-x1,y2-y1,&memDC,x1-179,y1-11,SRCCOPY);
	memDC.DeleteDC();
	pDC->DeleteDC();
	bmp.DeleteObject();
}
/***************************************************************************************************************************************/
void CMyDlg::Statistic() //计算总里程、总成本、时间、速度、路况
{
	//总里程
	D=Head->id*1.0/30;
	m_DistanceStr.Format("%.2lf",D);
	
	//速度在外部已改
	m_SpeedStr.Format("%.2lf",V);

	//时间
	CalculateTime();//计算时间
	m_WholeStr.Format("%.2lf",T);

	//总成本
	S=D*O*atof(m_FuelEdit)+atof(m_OtherEdit)+T*50;//时间费用有待考虑
	m_PrimeStr.Format("%.1lf",S);

	//路况
	if(V>=3.1 && V<=4.1)  {m_TrafficStr="拥挤";speed=60;}
	else				  {m_TrafficStr="流畅";speed=15;}

	UpdateData(FALSE);
}
/***************************************************************************************************************************************/
void CMyDlg::OnCarMenu() 
{
	FILE *rp;
	if((rp=fopen("CarA.txt","r"))==NULL) //还没有小车信息录入（即未按寻路与测试）	
	{
		MessageBox("暂无小车信息!","警告",MB_OK|MB_ICONSTOP);
		return;
	}
	else
	{
		fclose(rp);
		CCarDlg dlg;
		dlg.DoModal();
	}
}
/***************************************************************************************************************************************/
void CMyDlg::OnLawMenu() 
{
	CLawDlg dlg;
	dlg.DoModal();	
}
/***************************************************************************************************************************************/
BOOL CheckCool(ROUTE *left) //检测是否需要对剩余数组重新退火
{
	while(left->next != NULL) //往后遍历
	{
		if(oncrowd == FALSE && CheckTraffic(left->id,left->next->id) == FALSE) //该路段堵车
			return TRUE;
		left=left->next;
	}
	return FALSE;
}
/***************************************************************************************************************************************/
BOOL CheckClient(int first,int second) //查找first点和second点之间是否存在未配送到的客户结点
{

	for(int i=0;node[first].aroundid[i]!=0;i++)
		if(node[node[first].aroundid[i]].choose == CLIENTFLAG 
		&& node[node[first].aroundid[i]].been == FALSE) //如果在first结点周围发现未配送的客户结点
		{
			for(int j=0;node[second].aroundid[j]!=0;j++)
				if(node[first].aroundid[i] == node[second].aroundid[j])
					return TRUE;
		}

	return FALSE;
}
/***************************************************************************************************************************************/
BOOL CheckOncrowd(int r) //检测客户结点所在道路是否拥堵
{
	int s;
	for(int n=51;n<=71;n++)
	{
		if(node[n].choose == CLIENTFLAG || node[n].choose == STOREFLAG)
		{
			for(int i=0;(s=node[n].roadid[i])!=0;i++)
				if(s == r)
					return TRUE;
		}
	}
	return FALSE;
}

void CMyDlg::OnQueryMenu() 
{
	CQueryDlg dlg;
	dlg.DoModal();
}
