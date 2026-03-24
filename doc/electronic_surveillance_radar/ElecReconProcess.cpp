#include "ElecReconProcess.h"
#include "Coordinates/CoordTrans.h"
#include "model_export.h"
#include "Maths/MathFunctions.h"
#include <math.h>
CElecReconProcess::CElecReconProcess()
{
	m_SimplePassiveMode = 1;
	m_SimTime = 0;
	omp_init_lock(&Passivelock);
}


CElecReconProcess::~CElecReconProcess()
{
	omp_destroy_lock(&Passivelock);
	 m_RadarList.clear();
	 m_ComList.clear();
	 m_JammarList.clear();
	 m_FindRadiationList.clear();
	 m_TempTargetList.clear();
	 m_PassiveDetectList.clear();
	 m_RadiateList.clear();
}

void CElecReconProcess::SetJammers(std::vector<JAMMARPARA>& jam)
{
	m_JammarList = jam;
	for (size_t i = 0; i < m_JammarList.size(); i++)
	{
		ChangeAzFrom360To180(m_JammarList[i].m_JammaWork.m_AnteMBAz);
	}
}
void CElecReconProcess::SetRadars(std::vector<RADARDETECTPARA>& Radar)
{
	m_RadarList = Radar;
}
void CElecReconProcess::SetComs(std::vector<COMPARA>& Com)
{
	m_ComList = Com;
	for (size_t i = 0; i < m_ComList.size(); i++)
	{
		ChangeAzFrom360To180(m_ComList[i].m_ComEquip.m_AnteMBAz);
	}
}
bool CElecReconProcess::Initialize(const ELECRECONEQUIPPARA& ElecReconPara, const RADIATEPARA& RadiatePara, const  RADIATEPLATFORMWORKPARA2D& RadiatePlatformWorkPara2D)
{
	m_ElecReconEquipNum = 1;
	m_ElecReconEquipList[0] = ElecReconPara;
	m_ElecReconEquipList[0].m_AnteMBAz = RadiatePlatformWorkPara2D.m_CenterAz;
	m_ElecReconEquipList[0].m_AnteMBEl = RadiatePlatformWorkPara2D.m_CenterEl;
	ChangeAzFrom360To180(m_ElecReconEquipList[0].m_AnteMBAz);
	m_ElecReconEquipList[0].m_NoisePower = m_ElecReconEquipList[0].m_ReconSensitivity / pow(10.0, (14 + m_ElecReconEquipList[0].m_IntegratedRevLoss) / 10.0);

	m_ElecReconPos.lt = RadiatePlatformWorkPara2D.m_Lat*d2r;					//γ��
	m_ElecReconPos.ln = RadiatePlatformWorkPara2D.m_Lon*d2r;					//����

	LOCATION m_Location;
	double m_EarthHeight;
	m_Location.m_Lat = RadiatePlatformWorkPara2D.m_Lat;
	m_Location.m_Lon = RadiatePlatformWorkPara2D.m_Lon;
// 	m_Location.m_Lat = m_ElecReconPlatform.m_Lat;
// 	m_Location.m_Lon = m_ElecReconPlatform.m_Lon;

	GetEarthHeight(m_Location, m_EarthHeight);

	if (RadiatePlatformWorkPara2D.m_Alt<0)//������ø߶�С��0 ��Ĭ�����ڵ�ǰ����߶��� �ۼ�
	{
		m_ElecReconPos.ht = m_EarthHeight - RadiatePlatformWorkPara2D.m_Alt;
	}
	else//������ø߶ȴ���0 ��Ĭ��ȡ��ǰ����߶Ⱥ��״����ø߶ȵ����ֵ
		m_ElecReconPos.ht = MAX(RadiatePlatformWorkPara2D.m_Alt, m_EarthHeight);

//	m_ElecReconPos.ht = RadiatePlatformWorkPara2D.m_Alt > m_EarthHeight ? RadiatePlatformWorkPara2D.m_Alt : m_EarthHeight;
//	m_ElecReconPos.ht = m_ElecReconPlatform.m_Alt > m_EarthHeight ? m_ElecReconPlatform.m_Alt : m_EarthHeight;
	//m_ElecReconPos.ht = RadiatePlatformWorkPara2D.m_Alt;

	m_ElecReconAtt.roll = RadiatePlatformWorkPara2D.m_Phy *d2r;//��ת��
	m_ElecReconAtt.yaw = -RadiatePlatformWorkPara2D.m_Psi *d2r;  //ƫ����	
	m_ElecReconAtt.pitch = RadiatePlatformWorkPara2D.m_Theta *d2r;//������

	m_RadiatePara = RadiatePara;
	m_AntennaPara.m_AnteBeamType = 0;
	m_AntennaPara.m_AnteMBGain = ElecReconPara.m_AnteGain;
	m_AntennaPara.m_AnteSBGain = -60;
	m_AntennaPara.m_AzBeamWidth = ElecReconPara.m_AzBeamWidth;
	m_AntennaPara.m_ElBeamWidth = ElecReconPara.m_ElBeamWidth;

	TempTargetArrange(RadiatePlatformWorkPara2D);

	// ���ӻ�ȡ������Ż�����Added by ZMY		20220219
	EnvironmentNoisePower = 0;//��ȡ������Ż�������ǿ��
	BACKELECKEY radar_position;
	radar_position.m_WaveType = JudgeWaveType(RadiatePara.m_RadioFreqency);
	radar_position.m_Location.m_Lon = RadiatePlatformWorkPara2D.m_Lon;
	radar_position.m_Location.m_Lat = RadiatePlatformWorkPara2D.m_Lat;
	radar_position.m_Location.m_Alt = RadiatePlatformWorkPara2D.m_Alt;
	GetBackElecEnviron(radar_position, EnvironmentNoisePower);
	// ͨ����������������㻷������
	double c = 3.0e8; //���� LIGHTSPEED
	EnvironmentNoisePower = EnvironmentNoisePower * ElecReconPara.m_AnteGain * pow(c, 2) / (4 * PI * pow(RadiatePara.m_RadioFreqency, 2));


	return true;
}
bool CElecReconProcess::Initialize(const ELECRECONEQUIPPARA& ElecReconPara, const RADIATEPARA& RadiatePara, const  RADIATEPLATFORMWORKPARA2D2& RadiatePlatformWorkPara2D2)
{
	m_ElecReconEquipNum = 1;
	m_ElecReconEquipList[0] = ElecReconPara;
	m_ElecReconEquipList[0].m_AnteMBAz = RadiatePlatformWorkPara2D2.m_CenterAz;
	m_ElecReconEquipList[0].m_AnteMBEl = RadiatePlatformWorkPara2D2.m_CenterEl;
	ChangeAzFrom360To180(m_ElecReconEquipList[0].m_AnteMBAz);
	m_ElecReconEquipList[0].m_NoisePower = m_ElecReconEquipList[0].m_ReconSensitivity / pow(10.0, (14 + m_ElecReconEquipList[0].m_IntegratedRevLoss) / 10.0);

	m_ElecReconPos.lt = RadiatePlatformWorkPara2D2.m_Lat*d2r;					//γ��
	m_ElecReconPos.ln = RadiatePlatformWorkPara2D2.m_Lon*d2r;					//����

	LOCATION m_Location;
	double m_EarthHeight;
// 	m_Location.m_Lat = m_ElecReconPlatform.m_Lat;
// 	m_Location.m_Lon = m_ElecReconPlatform.m_Lon;
	m_Location.m_Lat = RadiatePlatformWorkPara2D2.m_Lat;
	m_Location.m_Lon = RadiatePlatformWorkPara2D2.m_Lon;

	GetEarthHeight(m_Location, m_EarthHeight);

	if (RadiatePlatformWorkPara2D2.m_Alt<0)//������ø߶�С��0 ��Ĭ�����ڵ�ǰ����߶��� �ۼ�
	{
		m_ElecReconPos.ht = m_EarthHeight - RadiatePlatformWorkPara2D2.m_Alt;
	}
	else//������ø߶ȴ���0 ��Ĭ��ȡ��ǰ����߶Ⱥ��״����ø߶ȵ����ֵ
		m_ElecReconPos.ht = MAX(RadiatePlatformWorkPara2D2.m_Alt, m_EarthHeight);

	//m_ElecReconPos.ht = m_ElecReconPlatform.m_Alt > m_EarthHeight ? m_ElecReconPlatform.m_Alt : m_EarthHeight;
	//m_ElecReconPos.ht = RadiatePlatformWorkPara3D.m_Alt;

	m_ElecReconAtt.roll = RadiatePlatformWorkPara2D2.m_Phy *d2r;//��ת��
	m_ElecReconAtt.yaw = -RadiatePlatformWorkPara2D2.m_Psi *d2r;  //ƫ����	
	m_ElecReconAtt.pitch = RadiatePlatformWorkPara2D2.m_Theta *d2r;//������

	m_RadiatePara = RadiatePara;

	m_AntennaPara.m_AnteBeamType = 0;
	m_AntennaPara.m_AnteMBGain = ElecReconPara.m_AnteGain;
	m_AntennaPara.m_AnteSBGain = -60;
	m_AntennaPara.m_AzBeamWidth = ElecReconPara.m_AzBeamWidth;
	m_AntennaPara.m_ElBeamWidth = ElecReconPara.m_ElBeamWidth;
	TempTargetArrange(RadiatePlatformWorkPara2D2);

	// ���ӻ�ȡ������Ż�����Added by ZMY		20220219
	EnvironmentNoisePower = 0;//��ȡ������Ż�������ǿ��
	BACKELECKEY radar_position;
	radar_position.m_WaveType = JudgeWaveType(RadiatePara.m_RadioFreqency);
	radar_position.m_Location.m_Lon = RadiatePlatformWorkPara2D2.m_Lon;
	radar_position.m_Location.m_Lat = RadiatePlatformWorkPara2D2.m_Lat;
	radar_position.m_Location.m_Alt = RadiatePlatformWorkPara2D2.m_Alt;
	GetBackElecEnviron(radar_position, EnvironmentNoisePower);
	// ͨ����������������㻷������
	double c = 3.0e8; //���� LIGHTSPEED
	EnvironmentNoisePower = EnvironmentNoisePower * ElecReconPara.m_AnteGain * pow(c, 2) / (4 * PI * pow(RadiatePara.m_RadioFreqency, 2));


	return true;
}

bool CElecReconProcess::Initialize(const ELECRECONEQUIPPARA& ElecReconPara, const RADIATEPARA& RadiatePara, const  RADIATEPLATFORMWORKPARA3D& RadiatePlatformWorkPara3D)
{
	m_ElecReconEquipNum = 1;
	m_ElecReconEquipList[0] = ElecReconPara;
	m_ElecReconEquipList[0].m_AnteMBAz = RadiatePlatformWorkPara3D.m_CenterAz;
	m_ElecReconEquipList[0].m_AnteMBEl = RadiatePlatformWorkPara3D.m_CenterEl;
	ChangeAzFrom360To180(m_ElecReconEquipList[0].m_AnteMBAz);
	m_ElecReconEquipList[0].m_NoisePower = m_ElecReconEquipList[0].m_ReconSensitivity / pow(10.0, (14 + m_ElecReconEquipList[0].m_IntegratedRevLoss) / 10.0);

	m_ElecReconPos.lt = RadiatePlatformWorkPara3D.m_Lat*d2r;					//γ��
	m_ElecReconPos.ln = RadiatePlatformWorkPara3D.m_Lon*d2r;					//����
	
	LOCATION m_Location;
	double m_EarthHeight;
// 	m_Location.m_Lat = m_ElecReconPlatform.m_Lat;
// 	m_Location.m_Lon = m_ElecReconPlatform.m_Lon;
	m_Location.m_Lat = RadiatePlatformWorkPara3D.m_Lat;
	m_Location.m_Lon = RadiatePlatformWorkPara3D.m_Lon;
	GetEarthHeight(m_Location, m_EarthHeight);

	if (RadiatePlatformWorkPara3D.m_Alt<0)//������ø߶�С��0 ��Ĭ�����ڵ�ǰ����߶��� �ۼ�
	{
		m_ElecReconPos.ht = m_EarthHeight - RadiatePlatformWorkPara3D.m_Alt;
	}
	else//������ø߶ȴ���0 ��Ĭ��ȡ��ǰ����߶Ⱥ��״����ø߶ȵ����ֵ
		m_ElecReconPos.ht = MAX(RadiatePlatformWorkPara3D.m_Alt, m_EarthHeight);
	//m_ElecReconPos.ht = RadiatePlatformWorkPara3D.m_Alt > m_EarthHeight ? RadiatePlatformWorkPara3D.m_Alt : m_EarthHeight;
//	m_ElecReconPos.ht = m_ElecReconPlatform.m_Alt > m_EarthHeight ? m_ElecReconPlatform.m_Alt : m_EarthHeight;
//	m_ElecReconPos.ht = RadiatePlatformWorkPara3D.m_Alt;

	m_ElecReconAtt.roll = RadiatePlatformWorkPara3D.m_Phy *d2r;//��ת��
	m_ElecReconAtt.yaw =- RadiatePlatformWorkPara3D.m_Psi *d2r;  //ƫ����	
	m_ElecReconAtt.pitch = RadiatePlatformWorkPara3D.m_Theta *d2r;//������

	m_RadiatePara = RadiatePara;

	m_AntennaPara.m_AnteBeamType = 0;
	m_AntennaPara.m_AnteMBGain = ElecReconPara.m_AnteGain;
	m_AntennaPara.m_AnteSBGain = -60;
	m_AntennaPara.m_AzBeamWidth = ElecReconPara.m_AzBeamWidth;
	m_AntennaPara.m_ElBeamWidth = ElecReconPara.m_ElBeamWidth;

	TempTargetArrange(RadiatePlatformWorkPara3D);

	// ���ӻ�ȡ������Ż�����Added by ZMY		20220219
	EnvironmentNoisePower = 0;//��ȡ������Ż�������ǿ��
	BACKELECKEY radar_position;
	radar_position.m_WaveType = JudgeWaveType(RadiatePara.m_RadioFreqency);
	radar_position.m_Location.m_Lon = RadiatePlatformWorkPara3D.m_Lon;
	radar_position.m_Location.m_Lat = RadiatePlatformWorkPara3D.m_Lat;
	radar_position.m_Location.m_Alt = RadiatePlatformWorkPara3D.m_Alt;
	GetBackElecEnviron(radar_position, EnvironmentNoisePower);

	// ͨ����������������㻷������
	double c = 3.0e8; //���� LIGHTSPEED
	EnvironmentNoisePower = EnvironmentNoisePower * ElecReconPara.m_AnteGain * pow(c, 2) / (4 * PI * pow(RadiatePara.m_RadioFreqency, 2));

	return true;
}

bool CElecReconProcess::Initialize(const ELECRECONPARA&  ElecReconPara)
{
	m_ElecReconPlatform = ElecReconPara.m_ElecReconPlatform;
	m_ElecReconEquipNum = ElecReconPara.m_ElecReconEquipNum;
	for (int i = 0; i < m_ElecReconEquipNum;i++)
	{
		m_ElecReconEquipList[i] = ElecReconPara.m_ElecReconEquipList[i];
		ChangeAzFrom360To180(m_ElecReconEquipList[i].m_AnteMBAz);
		m_ElecReconEquipList[i].m_NoisePower = m_ElecReconEquipList[i].m_ReconSensitivity / pow(10.0, (14 + m_ElecReconEquipList[i].m_IntegratedRevLoss)/10.0);
	}

	m_ElecReconPos.lt = m_ElecReconPlatform.m_Lat*d2r;					//γ��
	m_ElecReconPos.ln = m_ElecReconPlatform.m_Lon*d2r;					//����
	LOCATION m_Location;
	double m_EarthHeight;
	m_Location.m_Lat = m_ElecReconPlatform.m_Lat;
	m_Location.m_Lon = m_ElecReconPlatform.m_Lon;

	GetEarthHeight(m_Location, m_EarthHeight);


	m_ElecReconPos.ht = m_ElecReconPlatform.m_Alt>m_EarthHeight ? m_ElecReconPlatform.m_Alt:m_EarthHeight;

	m_ElecReconAtt.roll=m_ElecReconPlatform.m_Phy *d2r;//��ת��
	m_ElecReconAtt.yaw=-m_ElecReconPlatform.m_Psi *d2r;  //ƫ����	
	m_ElecReconAtt.pitch=m_ElecReconPlatform.m_Theta *d2r;//������

	return true;
}

bool CElecReconProcess::Initialize(PASSIVERADARPARA& PassiveRadarPara,
	std::vector<RADIATEDETECTPARA>&	RadiateList,
	std::vector<RADIATEDETECTPARA>&	GuideRadarList,
	std::vector<TARGETPARA>&        TargetList)
{

	m_RadarList.clear();
	m_ComList.clear();
	m_JammarList.clear();
	m_FindRadiationList.clear();
	m_TempTargetList.clear();
	m_PassiveDetectList.clear();
	m_RadiateList.clear();

	m_ElecReconPlatform = PassiveRadarPara.m_PassiveRadarPlatform;
	m_PassiveRadarEquip = PassiveRadarPara.m_PassiveRadarEquip;
	ChangeAzFrom360To180(m_PassiveRadarEquip.m_AnteMBAz);
	m_PassiveRadarEquip.m_NoisePower = m_PassiveRadarEquip.m_ReconSensitivity / pow(10.0, (14 + m_PassiveRadarEquip.m_IntegratedRevLoss) / 10.0);
	m_ElecReconPos.lt = m_ElecReconPlatform.m_Lat*d2r;					//γ��
	m_ElecReconPos.ln = m_ElecReconPlatform.m_Lon*d2r;					//����
	LOCATION m_Location;
	double m_EarthHeight;
	m_Location.m_Lat = m_ElecReconPlatform.m_Lat;
	m_Location.m_Lon = m_ElecReconPlatform.m_Lon;

	m_PassiveRadarEquip.m_AnteMBAz = 0.5*(m_PassiveRadarEquip.m_ScanStartAz + m_PassiveRadarEquip.m_ScanEndAz);
	m_PassiveRadarEquip.m_AnteMBEl = 0.5*(m_PassiveRadarEquip.m_ScanStartEl + m_PassiveRadarEquip.m_ScanEndEl);

	GetEarthHeight(m_Location, m_EarthHeight);

	if (m_ElecReconPlatform.m_Alt<0)//������ø߶�С��0 ��Ĭ�����ڵ�ǰ����߶��� �ۼ�
	{
		m_ElecReconPos.ht = m_EarthHeight - m_ElecReconPlatform.m_Alt;
	}
	else//������ø߶ȴ���0 ��Ĭ��ȡ��ǰ����߶Ⱥ��״����ø߶ȵ����ֵ
		m_ElecReconPos.ht = MAX(m_ElecReconPlatform.m_Alt, m_EarthHeight);


	m_ElecReconAtt.roll = m_ElecReconPlatform.m_Phy *d2r;//��ת��
	m_ElecReconAtt.yaw =- m_ElecReconPlatform.m_Psi *d2r;  //ƫ����	
	m_ElecReconAtt.pitch = m_ElecReconPlatform.m_Theta *d2r;//������

/////////////////////////////////////////////////////////////////////////////////////////////////////////
	GEOG_COORD tar;
	GEOG_COORD radar;
	OBSV_COORD polar;
	JamPowerCalculateParam JamParam;

	m_RadiateList = RadiateList;

	RADIATEDETECTPARA m_TargetRadiateDetectPara;
	for (int i = 0; i < TargetList.size();i++)
	{
		m_TargetRadiateDetectPara.m_TargetPara = TargetList[i];

		tar.ln = m_TargetRadiateDetectPara.m_TargetPara.m_TargetLon*d2r;
		tar.lt = m_TargetRadiateDetectPara.m_TargetPara.m_TargetLat*d2r;
		tar.ht = m_TargetRadiateDetectPara.m_TargetPara.m_TargetAlt;

		for (int j = 0; j < GuideRadarList.size();j++)
		{
			m_TargetRadiateDetectPara.m_RadiatePara = GuideRadarList[j].m_RadiatePara;

			radar.ln = GuideRadarList[j].m_TargetPara.m_TargetLon*d2r;
			radar.lt = GuideRadarList[j].m_TargetPara.m_TargetLat*d2r;
			radar.ht = GuideRadarList[j].m_TargetPara.m_TargetAlt;
			polar = GEOG2OBSV(tar, radar);//ע��ԭ��ѡ��
			/////////////////////////����ÿһ���״���Ŀ�괦�ķ���ǿ��
			JamParam.TransPower = GuideRadarList[j].m_RadiatePara.m_TransPower;
			JamParam.TransGain = GuideRadarList[j].m_RadiatePara.m_AnteGain;
			JamParam.TransLoss = GuideRadarList[j].m_RadiatePara.m_RadiateLoss;//������ͬ������Ƶ�������
			JamParam.RadarWavelength = LIGHTSPEED / GuideRadarList[j].m_RadiatePara.m_RadioFreqency;
			JamParam.JamRange = sqrt(polar.x*polar.x + polar.y*polar.y + polar.z*polar.z);
			JamParam.ReceiveLoss = 0;
			JamParam.Attenuation = GetAllBroadCastLostDB(m_WeatherPara, radar, tar, GuideRadarList[j].m_RadiatePara.m_RadioFreqency);
			JamParam.ReceiveGain = 0;
			JamParam.BandRatioFactor = m_TargetRadiateDetectPara.m_TargetPara.m_TargetRCS;

			//���㵽���״���ջ�ǰ�˵���������ֵ���ú�����REWS���ܿ���ʵ��
			m_TargetRadiateDetectPara.m_RadiatePara.m_TransPower = ReceiveJamPowerCalculate(JamParam);
			m_TargetRadiateDetectPara.m_RadiatePara.m_AnteGain = 0;
			m_TargetRadiateDetectPara.m_RadiatePara.m_RadiateLoss = 0;
			m_RadiateList.push_back(m_TargetRadiateDetectPara);
		}
	}
	if (m_SimplePassiveMode)
	{
		if (m_PassiveRadarEquip.m_ScanEndAz <= m_PassiveRadarEquip.m_ScanStartAz)  m_PassiveRadarEquip.m_ScanEndAz += 360.0;
		m_PassiveRadarEquip.m_AzBeamWidth = m_PassiveRadarEquip.m_ScanEndAz - m_PassiveRadarEquip.m_ScanStartAz;
		m_PassiveRadarEquip.m_ElBeamWidth = m_PassiveRadarEquip.m_ScanEndEl - m_PassiveRadarEquip.m_ScanStartEl;
		m_AnteMBAz = (m_PassiveRadarEquip.m_ScanStartAz+0.5*m_PassiveRadarEquip.m_AzBeamWidth) *d2r;             //�������ķ�λ���ָ�� ����
		m_AnteMBEl = (m_PassiveRadarEquip.m_ScanStartEl+0.5*m_PassiveRadarEquip.m_ElBeamWidth) *d2r;
	}
	else
	{
		BeamArrange(m_Beams);
	}


	return true;
}

long CElecReconProcess::BeamArrange(std::vector<BEAMSTATEPARA>& Beams)
{
	Beams.clear();

	//���ȼ��㷽λ�͸�����λ����
	short NumAZ(0), NumEL(0), NumBeam(0);
	short i(0), j(0);

	if (m_PassiveRadarEquip.m_ScanEndAz <= m_PassiveRadarEquip.m_ScanStartAz)  m_PassiveRadarEquip.m_ScanEndAz += 360.0;

	//if (m_RadarWorkPara.m_ScanEndAz > m_RadarWorkPara.m_ScanStartAz)
	//	NumAZ = ceil((m_RadarWorkPara.m_ScanEndAz - m_RadarWorkPara.m_ScanStartAz) / m_RadarAntennaPara.m_AzBeamWidth);
	//else
	//	NumAZ = ceil((360.0 + m_RadarWorkPara.m_ScanEndAz - m_RadarWorkPara.m_ScanStartAz) / m_RadarAntennaPara.m_AzBeamWidth);

	NumAZ = ceil((m_PassiveRadarEquip.m_ScanEndAz - m_PassiveRadarEquip.m_ScanStartAz) / m_PassiveRadarEquip.m_AzBeamWidth);
	NumEL = ceil((m_PassiveRadarEquip.m_ScanEndEl - m_PassiveRadarEquip.m_ScanStartEl) / m_PassiveRadarEquip.m_ElBeamWidth);
	NumBeam = NumAZ*NumEL;


	//���ŵĵ�����λ�ṹ��
	BEAMSTATEPARA SBeam;
	SBeam.m_AnteMBStayTime = 1;

	//����ÿ����λ�����õ�����̽�⺯����ͬʱ������ϲ�
	double BeamAz(0.0), BeamEl(0.0);

	if (0 == m_PassiveRadarEquip.m_ScanStartPos)
	{
		//ɨ������Ͽ�ʼ
		BeamAz = m_PassiveRadarEquip.m_ScanStartAz; // + 0.5*m_RadarAntennaPara.m_AzBeamWidth
		BeamEl = m_PassiveRadarEquip.m_ScanEndEl; // - 0.5*m_RadarAntennaPara.m_ElBeamWidth

		if (m_PassiveRadarEquip.m_ScanSequence == 0)
		{
			//ɨ��˳��Ϊ�����Һ�����
			for (i = 0; i < NumEL; i++)
			{
				BeamAz = m_PassiveRadarEquip.m_ScanStartAz;
				for (j = 0; j < NumAZ; j++)
				{
					SBeam.m_AnteMBAz = BeamAz;
					SBeam.m_AnteMBEl = BeamEl;

					Beams.push_back(SBeam);

					BeamAz += m_PassiveRadarEquip.m_AzBeamWidth;

					if (BeamAz > m_PassiveRadarEquip.m_ScanEndAz)
					{
						BeamAz = m_PassiveRadarEquip.m_ScanStartAz;
					}
				}

				BeamEl -= m_PassiveRadarEquip.m_ElBeamWidth;
			}
		}
		else
		{
			//ɨ��˳��Ϊ�����º�����
			for (i = 0; i < NumAZ; i++)
			{
				BeamEl = m_PassiveRadarEquip.m_ScanEndEl;
				for (j = 0; j < NumEL; j++)
				{
					SBeam.m_AnteMBAz = BeamAz;
					SBeam.m_AnteMBEl = BeamEl;

					Beams.push_back(SBeam);

					BeamEl -= m_PassiveRadarEquip.m_ElBeamWidth;

					if (BeamEl < m_PassiveRadarEquip.m_ScanStartEl)
					{
						BeamEl = m_PassiveRadarEquip.m_ScanEndEl;
					}
				}

				BeamAz += m_PassiveRadarEquip.m_AzBeamWidth;
			}
		}
	}
	else if (1 == m_PassiveRadarEquip.m_ScanStartPos)
	{
		//ɨ������Ͽ�ʼ
		BeamAz = m_PassiveRadarEquip.m_ScanEndAz;
		BeamEl = m_PassiveRadarEquip.m_ScanEndEl;

		if (m_PassiveRadarEquip.m_ScanSequence == 0)
		{
			//ɨ��˳��Ϊ�����Һ�����
			for (i = 0; i < NumEL; i++)
			{
				BeamAz = m_PassiveRadarEquip.m_ScanEndAz;
				for (j = 0; j < NumAZ; j++)
				{
					SBeam.m_AnteMBAz = BeamAz;
					SBeam.m_AnteMBEl = BeamEl;

					Beams.push_back(SBeam);

					BeamAz -= m_PassiveRadarEquip.m_AzBeamWidth;

					if (BeamAz < m_PassiveRadarEquip.m_ScanStartAz)
					{
						BeamAz = m_PassiveRadarEquip.m_ScanEndAz;
					}
				}

				BeamEl -= m_PassiveRadarEquip.m_ElBeamWidth;
			}
		}
		else
		{
			//ɨ��˳��Ϊ�����º�����
			for (i = 0; i < NumAZ; i++)
			{
				BeamEl = m_PassiveRadarEquip.m_ScanEndEl;
				for (j = 0; j < NumEL; j++)
				{
					SBeam.m_AnteMBAz = BeamAz;
					SBeam.m_AnteMBEl = BeamEl;

					Beams.push_back(SBeam);

					BeamEl -= m_PassiveRadarEquip.m_ElBeamWidth;

					if (BeamEl < m_PassiveRadarEquip.m_ScanStartEl)
					{
						BeamEl = m_PassiveRadarEquip.m_ScanEndEl;
					}
				}

				BeamAz -= m_PassiveRadarEquip.m_AzBeamWidth;
			}
		}
	}
	else if (2 == m_PassiveRadarEquip.m_ScanStartPos)
	{
		//ɨ������¿�ʼ
		BeamAz = m_PassiveRadarEquip.m_ScanEndAz;
		BeamEl = m_PassiveRadarEquip.m_ScanStartEl;

		if (m_PassiveRadarEquip.m_ScanSequence == 0)
		{
			//ɨ��˳��Ϊ�����Һ�����
			for (i = 0; i < NumEL; i++)
			{
				BeamAz = m_PassiveRadarEquip.m_ScanEndAz;
				for (j = 0; j < NumAZ; j++)
				{
					SBeam.m_AnteMBAz = BeamAz;
					SBeam.m_AnteMBEl = BeamEl;

					Beams.push_back(SBeam);

					BeamAz -= m_PassiveRadarEquip.m_AzBeamWidth;

					if (BeamAz < m_PassiveRadarEquip.m_ScanStartAz)
					{
						BeamAz = m_PassiveRadarEquip.m_ScanEndAz;
					}
				}

				BeamEl += m_PassiveRadarEquip.m_ElBeamWidth;
			}
		}
		else
		{
			//ɨ��˳��Ϊ�����º�����
			for (i = 0; i < NumAZ; i++)
			{
				BeamEl = m_PassiveRadarEquip.m_ScanStartEl;
				for (j = 0; j < NumEL; j++)
				{
					SBeam.m_AnteMBAz = BeamAz;
					SBeam.m_AnteMBEl = BeamEl;

					Beams.push_back(SBeam);

					BeamEl += m_PassiveRadarEquip.m_ElBeamWidth;

					if (BeamEl > m_PassiveRadarEquip.m_ScanEndEl)
					{
						BeamEl = m_PassiveRadarEquip.m_ScanStartEl;
					}
				}

				BeamAz -= m_PassiveRadarEquip.m_AzBeamWidth;
			}
		}
	}
	else if (3 == m_PassiveRadarEquip.m_ScanStartPos)
	{
		//ɨ������¿�ʼ
		BeamAz = m_PassiveRadarEquip.m_ScanStartAz;
		BeamEl = m_PassiveRadarEquip.m_ScanStartEl;

		if (m_PassiveRadarEquip.m_ScanSequence == 0)
		{
			//ɨ��˳��Ϊ�����Һ�����
			for (i = 0; i < NumEL; i++)
			{
				BeamAz = m_PassiveRadarEquip.m_ScanStartAz;
				for (j = 0; j < NumAZ; j++)
				{
					SBeam.m_AnteMBAz = BeamAz;
					SBeam.m_AnteMBEl = BeamEl;

					Beams.push_back(SBeam);

					BeamAz += m_PassiveRadarEquip.m_AzBeamWidth;

					if (BeamAz > m_PassiveRadarEquip.m_ScanEndAz)
					{
						BeamAz = m_PassiveRadarEquip.m_ScanStartAz;
					}
				}

				BeamEl += m_PassiveRadarEquip.m_ElBeamWidth;
			}
		}
		else
		{
			//ɨ��˳��Ϊ�����º�����
			for (i = 0; i < NumAZ; i++)
			{
				BeamEl = m_PassiveRadarEquip.m_ScanStartEl;
				for (j = 0; j < NumEL; j++)
				{
					SBeam.m_AnteMBAz = BeamAz;
					SBeam.m_AnteMBEl = BeamEl;

					Beams.push_back(SBeam);

					BeamEl += m_PassiveRadarEquip.m_ElBeamWidth;

					if (BeamEl > m_PassiveRadarEquip.m_ScanEndEl)
					{
						BeamEl = m_PassiveRadarEquip.m_ScanStartEl;
					}
				}

				BeamAz += m_PassiveRadarEquip.m_AzBeamWidth;
			}
		}
	}

	return Beams.size();
}



short CElecReconProcess::JudgeWaveType(double _RadioFreqency)
{
	double temp = _RadioFreqency / 1000000000;
	if (temp < 0.23)
	{
		return 0;   //	Ƶ�� < 230 MHz
	}
	else if (temp >= 0.23 && temp < 1)
	{
		return 1;	//	P����	0.23-1 GHz
	}
	else if (temp >= 1 && temp < 2)
	{
		return 2;	//	L����	1-2 GHz
	}
	else if (temp >= 2 && temp < 4)
	{
		return 3;	//	S����	2-4 GHz
	}
	else if (temp >= 4 && temp < 8)
	{
		return 4;	//	C����	4-8 GHz
	}
	else if (temp >= 8 && temp < 12)
	{
		return 5;	//	X����	8-12 GHz
	}
	else if (temp >= 12 && temp < 18)
	{
		return 6;	//	Ku����	12-18 GHz
	}
	else if (temp >= 18 && temp < 27)
	{
		return 7;	//	K����	18-27 GHz
	}
	else if (temp >= 27 && temp < 40)
	{
		return 8;	//	Ka����	27-40 GHz
	}
	else if (temp >= 40 && temp < 60)
	{
		return 9;	//	U����	40-60 GHz
	}
	else if (temp >= 60 && temp < 80)
	{
		return 10;	//	V����	60-80 GHz
	}
	else if (temp >= 80 && temp < 100)
	{
		return 11;	//	W����	80-100 GHz
	}
	else
	{
		return 12;	//	Ƶ�� > 100GHz
	}
}

long CElecReconProcess::TempTargetArrange(const RADIATEPLATFORMWORKPARA2D& RadiatePlatformWorkPara2D)
{
	m_TempTargetList.clear();

	OBSV_COORD m_ObsvCoord;
	MSLB_COORD m_MLSBCoord;
	TEMPTARGETPARA m_TempTargetPara;
	NumAZ = ceil(fabs(RadiatePlatformWorkPara2D.m_EndLat - RadiatePlatformWorkPara2D.m_StartLat) / RadiatePlatformWorkPara2D.m_LatSeperate);
	NumEL = ceil(fabs(RadiatePlatformWorkPara2D.m_EndLon - RadiatePlatformWorkPara2D.m_StartLon) / RadiatePlatformWorkPara2D.m_LonSeperate);

	for (int i = 0; i < NumAZ; i++)
	{
		for (int j = 0; j < NumEL; j++)
		{
			m_TempTargetPara.m_TargetPosG.lt = RadiatePlatformWorkPara2D.m_StartLat +
				(i + 0.5)*(RadiatePlatformWorkPara2D.m_EndLat - RadiatePlatformWorkPara2D.m_StartLat>0) ? RadiatePlatformWorkPara2D.m_LatSeperate : -RadiatePlatformWorkPara2D.m_LatSeperate;
			m_TempTargetPara.m_TargetPosG.ln = RadiatePlatformWorkPara2D.m_StartLon +
				(j + 0.5)*(RadiatePlatformWorkPara2D.m_EndLon - RadiatePlatformWorkPara2D.m_StartLon>0) ? RadiatePlatformWorkPara2D.m_LonSeperate : -RadiatePlatformWorkPara2D.m_LonSeperate;
			m_TempTargetPara.m_TargetPosG.lt *= d2r;
			m_TempTargetPara.m_TargetPosG.ln *= d2r;
			m_TempTargetPara.m_TargetPosG.ht = RadiatePlatformWorkPara2D.m_Alt;
			m_TempTargetPara.m_TargetRCS = RadiatePlatformWorkPara2D.m_TargetRCS;
			m_TempTargetPara.m_RangeResolution = 1000000;
			m_ObsvCoord = GEOG2OBSV(m_TempTargetPara.m_TargetPosG, m_ElecReconPos);
			m_TempTargetPara.m_TargetPosO = OBSVPOLAR(m_ObsvCoord);
			m_TempTargetPara.m_Detect = false;

			m_MLSBCoord = OBSV2MSLB(m_ObsvCoord, m_ElecReconAtt);
			m_TempTargetPara.m_TargetPosM = MSLBPOLAR(m_MLSBCoord);

			m_TempTargetList.push_back(m_TempTargetPara);
		}
	}
	return m_TempTargetList.size();
}
long CElecReconProcess::TempTargetArrange(const RADIATEPLATFORMWORKPARA2D2 m_RadiatePlatformWorkPara2D2)
{
	m_TempTargetList.clear();

	//���ȼ��㷽λ�͸�����λ����
	long  NumBeam(0);
	short i(0);
	OBSV_COORD m_ObsvCoord;
	MSLB_COORD m_MLSBCoord;
	TEMPTARGETPARA m_TempTargetPara;
	NumAZ = ceil(/*m_ElecReconEquipList[0].m_AzBeamWidth*/360 / m_RadiatePlatformWorkPara2D2.m_AzSeperate);

	NumBeam = NumAZ/**NumEL*/;

	//ɨ��˳��Ϊ�����º�����
	for (i = 0; i < NumAZ; i++)
	{
		m_TempTargetPara.m_TargetPosM.az = (m_ElecReconEquipList[0].m_AnteMBAz - m_ElecReconEquipList[0].m_AzBeamWidth*0.5 + (i + 0.5)*m_RadiatePlatformWorkPara2D2.m_AzSeperate)*d2r;
		m_TempTargetPara.m_TargetPosM.el = 0;
		m_TempTargetPara.m_TargetPosM.rt = 1000000;//����1000����ݼ�
		ChangeAzFrom360To180(m_TempTargetPara.m_TargetPosM.az, true);
		m_MLSBCoord = POLARMSLB(m_TempTargetPara.m_TargetPosM);
		m_ObsvCoord = MSLB2OBSV(m_MLSBCoord, m_ElecReconAtt);
		m_TempTargetPara.m_TargetPosO = OBSVPOLAR(m_ObsvCoord);

// 		m_TempTargetPara.m_TargetPosO.az = (m_ElecReconEquipList[0].m_AnteMBAz - m_ElecReconEquipList[0].m_AzBeamWidth*0.5 + (i + 0.5)*m_RadiatePlatformWorkPara2D2.m_AzSeperate)*d2r;
// 		m_TempTargetPara.m_TargetPosO.el = 0;
// 		m_TempTargetPara.m_TargetPosO.rt = 1000000;//����1000����ݼ�
		m_ObsvCoord = POLAROBSV(m_TempTargetPara.m_TargetPosO);
		m_TempTargetPara.m_TargetPosG = OBSV2GEOG(m_ObsvCoord, m_ElecReconPos);
		m_TempTargetPara.m_RangeResolution = m_RadiatePlatformWorkPara2D2.m_RangeResolution;
		m_TempTargetPara.m_TarRevHlt = m_RadiatePlatformWorkPara2D2.m_TarRevHlt;
		m_TempTargetList.push_back(m_TempTargetPara);
	}
	return m_TempTargetList.size();
}
long CElecReconProcess::TempTargetArrange(const RADIATEPLATFORMWORKPARA3D m_RadiatePlatformWorkPara3D)
{
	m_TempTargetList.clear();

	//���ȼ��㷽λ�͸�����λ����
	long  NumBeam(0);
	short i(0), j(0);
	OBSV_COORD m_ObsvCoord;
	MSLB_COORD m_MLSBCoord;
	TEMPTARGETPARA m_TempTargetPara;
	NumAZ = ceil(/*m_ElecReconEquipList[0].m_AzBeamWidth*/360 / m_RadiatePlatformWorkPara3D.m_AzSeperate);
	NumEL = ceil(m_ElecReconEquipList[0].m_ElBeamWidth / m_RadiatePlatformWorkPara3D.m_ElSeperate);
	NumBeam = NumAZ*NumEL;

	//ɨ��˳��Ϊ�����º�����
	for (i = 0; i < NumAZ; i++)
	{
		for (j = 0; j < NumEL; j++)
		{

			m_TempTargetPara.m_TargetPosM.az = (m_ElecReconEquipList[0].m_AnteMBAz - m_ElecReconEquipList[0].m_AzBeamWidth*0.5 + (i + 0.5)*m_RadiatePlatformWorkPara3D.m_AzSeperate)*d2r;
			m_TempTargetPara.m_TargetPosM.el = (m_ElecReconEquipList[0].m_AnteMBEl - m_ElecReconEquipList[0].m_ElBeamWidth*0.5 + (j + 0.5)*m_RadiatePlatformWorkPara3D.m_ElSeperate)*d2r;
			m_TempTargetPara.m_TargetPosM.rt = 1000000;//����1000����ݼ�
			ChangeAzFrom360To180(m_TempTargetPara.m_TargetPosM.az, true);

			m_MLSBCoord = POLARMSLB(m_TempTargetPara.m_TargetPosM);
			m_ObsvCoord = MSLB2OBSV(m_MLSBCoord, m_ElecReconAtt);
			m_TempTargetPara.m_TargetPosO = OBSVPOLAR(m_ObsvCoord);

// 			m_TempTargetPara.m_TargetPosO.az = (m_ElecReconEquipList[0].m_AnteMBAz - m_ElecReconEquipList[0].m_AzBeamWidth*0.5 + (i + 0.5)*m_RadiatePlatformWorkPara3D.m_AzSeperate)*d2r;
// 			m_TempTargetPara.m_TargetPosO.el = (m_ElecReconEquipList[0].m_AnteMBEl - m_ElecReconEquipList[0].m_ElBeamWidth*0.5 + (j + 0.5)*m_RadiatePlatformWorkPara3D.m_ElSeperate)*d2r;
// 			m_TempTargetPara.m_TargetPosO.rt = 1000000;//����1000����ݼ�
			m_ObsvCoord = POLAROBSV(m_TempTargetPara.m_TargetPosO);
			m_TempTargetPara.m_TargetPosG = OBSV2GEOG(m_ObsvCoord, m_ElecReconPos);
			m_TempTargetPara.m_RangeResolution = m_RadiatePlatformWorkPara3D.m_RangeResolution;
			m_TempTargetList.push_back(m_TempTargetPara);
		}
	}
	return m_TempTargetList.size();
}
void CElecReconProcess::PreJamPowerCal()
{
	GEOG_COORD tar;
	OBSV_COORD radar;
	MSLB_COORD plane;
	MSLB_POLAR polar;
	EULER_ANGLE m_RadarAtt, pose;
	JamPowerCalculateParam JamParam;
	long  m, i, j;
	bool AzFlag, ElFlag;
	double JamTxGain(0.0), JamRxGain(0.0);
	double EchoPower(0.0);


	for (m = 0; m < m_ElecReconEquipNum; m++)
	{
		///////////////////////////////////
		m_ElecReconEquipList[m].m_JamPower = 0;
		double m_TempEl = m_ElecReconEquipList[m].m_AnteMBEl*d2r;
		//��Գ����еĸ�����Ž��д���
		for (i = 0; i < m_JammarList.size(); i++)
		{
			//ȷ���ø��Ż��Ѿ���������
			if (m_JammarList[i].m_JammaWork.m_State == 0) continue;

			tar.ln = m_JammarList[i].m_JammaWork.m_Lon*d2r;
			tar.lt = m_JammarList[i].m_JammaWork.m_Lat*d2r;
			tar.ht = m_JammarList[i].m_JammaWork.m_Alt;

			radar = GEOG2OBSV(m_ElecReconPos, tar);//ע��ԭ��ѡ�� �������������Ը��Ż��Ŀռ�λ��
			
			double m_JammerMultipathCoef = CalMultiPathCoef2(m_ElecReconPos, tar, m_AntennaPara, m_TempEl, m_ElecReconAtt, m_JammarList[i].m_JammarEquip->m_RadioFreqency);//��ʱʹ�ø��Ż�������Ƶ��
			//ѡȡ���Ż�����ƽ̨��Ӧ����̬��
			pose.yaw =- m_JammarList[i].m_JammaWork.m_Psi*d2r;
			pose.pitch = m_JammarList[i].m_JammaWork.m_Theta*d2r;
			pose.roll = m_JammarList[i].m_JammaWork.m_Phy*d2r;

			plane = OBSV2MSLB(radar, pose);

			polar = MSLBPOLAR(plane);
			//if (polar.az < 0.0)	polar.az += 2.0 * PI;
			/////////������������Ƿ��ڸ��Ż�������Χ��
			AzFlag = ComputeAzIntersectionAngle(polar.az, m_JammarList[i].m_JammaWork.m_AnteMBAz*d2r, true) <= m_JammarList[i].m_JammarAntenna.m_AzBeamWidth*d2r*0.50;
			ElFlag = fabs(polar.el - m_JammarList[i].m_JammaWork.m_AnteMBEl*d2r) <= m_JammarList[i].m_JammarAntenna.m_ElBeamWidth*d2r*0.50;

			if (!AzFlag || !ElFlag)
			{
				continue;
			}
			//���������δ��ȷ���Ż��԰�����ֵ������-40.0dB��Ĭ��ֵ���д���(������������СԼ60dB)
			//JamTxGain = (AzFlag && ElFlag) ? m_JammarList[i].m_JammarAntenna.m_AnteGain : -40.0;
			JamTxGain = m_JammarList[i].m_JammarAntenna.m_AnteGain;
			/////////////////////////////////////////////////////////////////

			radar = GEOG2OBSV(tar, m_ElecReconPos);
			plane = OBSV2MSLB(radar, m_ElecReconAtt);
			polar = MSLBPOLAR(plane);
			//if (polar.az < 0.0)	polar.az += 2.0 * PI;
			/////////////////////////////////������Ż��Ƿ��ڵ����������߷�Χ��
			AzFlag = ComputeAzIntersectionAngle(polar.az, m_ElecReconEquipList[m].m_AnteMBAz*d2r, true) <= m_ElecReconEquipList[m].m_AzBeamWidth* d2r*0.50;
			ElFlag = fabs(polar.el - m_ElecReconEquipList[m].m_AnteMBEl* d2r) <= m_ElecReconEquipList[m].m_ElBeamWidth* d2r*0.50;
			if (!AzFlag || !ElFlag)
			{
				continue;
			}
			JamRxGain = m_ElecReconEquipList[m].m_AnteGain;

			if (!InterVisibility(m_ElecReconPos, tar))
			{
				continue;
			}

			//��һ������Ź���
			//ÿһ������ƽ̨�п���Я����������豸
			for (j = 0; j < m_JammarList[i].m_JammarEquipNum; j++)
			{
				double m_FC = FrequencyCover(m_ElecReconEquipList[m].m_BandWidthDownLimit, m_ElecReconEquipList[m].m_BandWidthUpLimit, m_JammarList[i].m_JammarEquip[j].m_RadioFreqency, m_JammarList[i].m_JammarEquip[j].m_BandWidth);
				if (m_FC<eps)
					continue;
				//��ȡ��Ӧ�ĸ��Ų���
				JamParam.TransPower = m_JammarList[i].m_JammarEquip[j].m_TransPower;
				JamParam.TransGain = JamTxGain;
				JamParam.TransLoss = m_JammarList[i].m_JammarEquip[j].m_RadiateLoss;//������ͬ������Ƶ�������
				JamParam.RadarWavelength = LIGHTSPEED / m_JammarList[i].m_JammarEquip[j].m_RadioFreqency;
				JamParam.JamRange = polar.rt;
				JamParam.ReceiveLoss = m_ElecReconEquipList[m].m_IntegratedRevLoss;
				//JamParam.Attenuation = 1.8;//���������Ĭ�ϴ����������ֵ
				JamParam.Attenuation = GetAllBroadCastLostDB(m_WeatherPara, tar, m_ElecReconPos, m_JammarList[i].m_JammarEquip[j].m_RadioFreqency);
				JamParam.ReceiveGain = JamRxGain;
				JamParam.BandRatioFactor = m_FC/*1.2*CFAParam.FMBandwidth / CFAParam.JamBandwidth;*/;

				//���㵽���״���ջ�ǰ�˵���������ֵ���ú�����REWS���ܿ���ʵ��
				EchoPower = ReceiveJamPowerCalculate(JamParam);

				EchoPower *= m_JammerMultipathCoef;
				//ͳһ������ѹ�ƴ�������
				if (EchoPower > eps)
				{
					m_ElecReconEquipList[m].m_JamPower += EchoPower;
				}
			}
		}
	}
}

void CElecReconProcess::ScopeAdvance2D()
{
	double m_FC = FrequencyCover(m_ElecReconEquipList[0].m_BandWidthDownLimit, m_ElecReconEquipList[0].m_BandWidthUpLimit, m_RadiatePara.m_RadioFreqency, m_RadiatePara.m_BandWidth);

	if (m_FC < eps)
		return;

	PreJamPowerCal();
	short n(0);
	OBSV_COORD m_ObsvCoord;
	PowerCalculateParam PCParam;
	FINDTARGET DetObject;
	CFAgilityParam CFAParam;
	JamPowerCalculateParam JamParam;
	double EchoPower(0.0), SNR(0.0);
	bool AzFlag, ElFlag;
	short TargetNum = m_TempTargetList.size();
	GEOG_COORD tar;
	OBSV_COORD radar;
	MSLB_COORD plane;
	MSLB_POLAR polar, TarPosPolar, FTPolar;
	EULER_ANGLE pose;

	for (n = 0; n < TargetNum; n++)
	{
		radar = GEOG2OBSV(m_TempTargetList[n].m_TargetPosG, m_ElecReconPos);
		plane = OBSV2MSLB(radar, m_ElecReconAtt);
		polar = MSLBPOLAR(plane);

		//if (polar.az < 0.0)	polar.az += 2.0 * PI;

		AzFlag = ComputeAzIntersectionAngle(polar.az, m_ElecReconEquipList[0].m_AnteMBAz*d2r, true) <= m_ElecReconEquipList[0].m_AzBeamWidth*d2r*0.50;
		ElFlag = fabs(polar.el - m_ElecReconEquipList[0].m_AnteMBEl*d2r) <= m_ElecReconEquipList[0].m_ElBeamWidth*d2r*0.50;

		if (!AzFlag || !ElFlag)
		{
			continue;
		}
		if (!InterVisibility(m_ElecReconPos, m_TempTargetList[n].m_TargetPosG))
		{
			continue;
		}
		double m_TempEl = m_ElecReconEquipList[0].m_AnteMBEl*d2r;
		double m_MultipathCoef = CalMultiPathCoef2(m_ElecReconPos, m_TempTargetList[n].m_TargetPosG, m_AntennaPara, m_TempEl, m_ElecReconAtt, m_RadiatePara.m_RadioFreqency);
		//��ȡ��Ӧ�ĸ��Ų���
		JamParam.TransPower = m_RadiatePara.m_TransPower;
		JamParam.TransGain = m_RadiatePara.m_AnteGain;
		JamParam.TransLoss = m_RadiatePara.m_RadiateLoss;//������ͬ������Ƶ�������
		JamParam.RadarWavelength = LIGHTSPEED / m_RadiatePara.m_RadioFreqency;
		JamParam.JamRange = polar.rt;
		JamParam.ReceiveLoss = m_ElecReconEquipList[0].m_IntegratedRevLoss;
		//JamParam.Attenuation = 1.8;//���������Ĭ�ϴ����������ֵ
		JamParam.Attenuation = GetAllBroadCastLostDB(m_WeatherPara, m_TempTargetList[n].m_TargetPosG, m_ElecReconPos, m_RadiatePara.m_RadioFreqency);
		JamParam.ReceiveGain = m_ElecReconEquipList[0].m_AnteGain;
		JamParam.BandRatioFactor = m_FC;
		EchoPower = ReceiveJamPowerCalculate(JamParam);
		if (EchoPower < eps)
		{
			continue;
		}
		EchoPower *= m_MultipathCoef;
		//�����Ÿɱ�

		SNR = 10.0*log10(EchoPower / (m_ElecReconEquipList[0].m_NoisePower + m_ElecReconEquipList[0].m_JamPower + EnvironmentNoisePower) + eps);
		if (SNR > 14)//��Ӧ�����ʴ���50%
		{
			m_TempTargetList[n].m_Detect = true;
		}
	}
}

void CElecReconProcess::ScopeAdvance2D2()
{
	//ִ��Ŀ��ز��������
	double m_FC = FrequencyCover(m_ElecReconEquipList[0].m_BandWidthDownLimit, m_ElecReconEquipList[0].m_BandWidthUpLimit, m_RadiatePara.m_RadioFreqency, m_RadiatePara.m_BandWidth);
	if (m_FC < eps)
	{
		for (int n = 0; n < m_TempTargetList.size(); n++)
			m_TempTargetList[n].m_TargetPosO.rt = 0;
		return;
	}

//	PreJamPowerCal();

	short n(0), TargetNum = m_TempTargetList.size();

#pragma omp parallel for
	for (n = 0; n < TargetNum; n++)
	{
		double R2;
		OBSV_COORD m_ObsvCoord;
		PowerCalculateParam PCParam;
		FINDTARGET DetObject;
		CFAgilityParam CFAParam;
		JamPowerCalculateParam JamParam;
		double EchoPower(0.0), SNR(0.0);
		GEOG_COORD tar;
		OBSV_COORD radar;
		MSLB_COORD plane;
		MSLB_POLAR polar, TarPosPolar, FTPolar;
		EULER_ANGLE pose;

		double Seperate;
		double m_MaxLat, m_MaxLon, m_MinLat, m_MinLon;
		m_MaxLat = m_TempTargetList[n].m_TargetPosG.lt;
		m_MaxLon = m_TempTargetList[n].m_TargetPosG.ln;
		m_MinLat = m_ElecReconPos.lt;
		m_MinLon = m_ElecReconPos.ln;
		Seperate = m_TempTargetList[n].m_TargetPosO.rt;
		while (Seperate > m_TempTargetList[n].m_RangeResolution)
		{
			Seperate = m_TempTargetList[n].m_TargetPosO.rt;
			m_TempTargetList[n].m_TargetPosG.ln = 0.5*(m_MaxLon + m_MinLon);
			m_TempTargetList[n].m_TargetPosG.lt = 0.5*(m_MaxLat + m_MinLat);
			m_TempTargetList[n].m_TargetPosG.ht = m_TempTargetList[n].m_TarRevHlt;
			m_ObsvCoord = GEOG2OBSV(m_TempTargetList[n].m_TargetPosG, m_ElecReconPos);
			m_TempTargetList[n].m_TargetPosO = OBSVPOLAR(m_ObsvCoord);
			Seperate = fabs(Seperate - m_TempTargetList[n].m_TargetPosO.rt);

			R2 = 4120 * (pow(m_TempTargetList[n].m_TargetPosG.ht, 0.5) + pow(m_ElecReconPos.ht, 0.5));
			if (m_TempTargetList[n].m_TargetPosG.ht < 0 || R2 < m_TempTargetList[n].m_TargetPosO.rt || !InterVisibility(m_ElecReconPos, m_TempTargetList[n].m_TargetPosG))
			{
				m_MaxLat = m_TempTargetList[n].m_TargetPosG.lt;
				m_MaxLon = m_TempTargetList[n].m_TargetPosG.ln;
				continue;
			}

			//////////////�Զ�������ջ�ָ��
			m_ElecReconEquipList[0].m_AnteMBAz = m_TempTargetList[n].m_TargetPosO.az*r2d;
			m_ElecReconEquipList[0].m_AnteMBEl = m_TempTargetList[n].m_TargetPosO.el*r2d;
			ChangeAzFrom360To180(m_ElecReconEquipList[0].m_AnteMBAz, false);
			PreJamPowerCal();
			double m_TempEl = m_ElecReconEquipList[0].m_AnteMBEl*d2r;
			double m_MultipathCoef = CalMultiPathCoef2(m_ElecReconPos, m_TempTargetList[n].m_TargetPosG, m_AntennaPara, m_TempEl, m_ElecReconAtt, m_RadiatePara.m_RadioFreqency);

			//��ȡ��Ӧ�ĸ��Ų���
			JamParam.TransPower = m_RadiatePara.m_TransPower;
			JamParam.TransGain = m_RadiatePara.m_AnteGain;
			JamParam.TransLoss = m_RadiatePara.m_RadiateLoss;//������ͬ������Ƶ�������
			JamParam.RadarWavelength = LIGHTSPEED / m_RadiatePara.m_RadioFreqency;
			JamParam.JamRange = m_TempTargetList[n].m_TargetPosO.rt;
			JamParam.ReceiveLoss = m_ElecReconEquipList[0].m_IntegratedRevLoss;
			//JamParam.Attenuation = 1.8;//���������Ĭ�ϴ����������ֵ
			JamParam.Attenuation = GetAllBroadCastLostDB(m_WeatherPara, m_TempTargetList[n].m_TargetPosG, m_ElecReconPos, m_RadiatePara.m_RadioFreqency);
			JamParam.ReceiveGain = m_ElecReconEquipList[0].m_AnteGain;
			JamParam.BandRatioFactor = m_FC;
			EchoPower = ReceiveJamPowerCalculate(JamParam);

			EchoPower *= m_MultipathCoef;
			//�����Ÿɱ�

			SNR = 10.0*log10(EchoPower / (m_ElecReconEquipList[0].m_NoisePower + m_ElecReconEquipList[0].m_JamPower + EnvironmentNoisePower) + eps);
			if (SNR > 14)//��Ӧ�����ʴ���50%
			{
				m_MinLat = m_TempTargetList[n].m_TargetPosG.lt;
				m_MinLon = m_TempTargetList[n].m_TargetPosG.ln;
			}
			else
			{
				m_MaxLat = m_TempTargetList[n].m_TargetPosG.lt;
				m_MaxLon = m_TempTargetList[n].m_TargetPosG.ln;
			}
		}
	}
}
//̽�ⷶΧ����������
void CElecReconProcess::ScopeAdvance3D()
{
	//ִ��Ŀ��ز��������
	double m_FC = FrequencyCover(m_ElecReconEquipList[0].m_BandWidthDownLimit, m_ElecReconEquipList[0].m_BandWidthUpLimit, m_RadiatePara.m_RadioFreqency, m_RadiatePara.m_BandWidth);
	if (m_FC < eps)
	{
		for (int n = 0; n < m_TempTargetList.size(); n++)
			m_TempTargetList[n].m_TargetPosO.rt = 0;
		return;
	}

//	PreJamPowerCal();

	short n(0), TargetNum = m_TempTargetList.size();

#pragma omp parallel for
	for (n = 0; n < TargetNum; n++)
	{
		double  R2;
		OBSV_COORD m_ObsvCoord;
		PowerCalculateParam PCParam;
		FINDTARGET DetObject;
		CFAgilityParam CFAParam;
		JamPowerCalculateParam JamParam;
		double EchoPower(0.0), SNR(0.0);
		GEOG_COORD tar;
		OBSV_COORD radar;
		MSLB_COORD plane;
		MSLB_POLAR polar, TarPosPolar, FTPolar;
		EULER_ANGLE pose;
// 		for (R1 = m_TempTargetList[n].m_TargetPosO.rt; R1 > 0; R1 -= m_TempTargetList[n].m_RangeResolution)
// 		{
// 			m_TempTargetList[n].m_TargetPosO.rt = R1;
// 			m_ObsvCoord = POLAROBSV(m_TempTargetList[n].m_TargetPosO);
// 			m_TempTargetList[n].m_TargetPosG = OBSV2GEOG(m_ObsvCoord, m_ElecReconPos);
// 			R2 = 4120 * (pow(m_TempTargetList[n].m_TargetPosG.ht, 0.5) + pow(m_ElecReconPos.ht, 0.5));
// 			if (R2 < R1)
// 			{
// 				continue;
// 			}
// 			if (!InterVisibility(m_ElecReconPos, m_TempTargetList[n].m_TargetPosG))
// 			{
// 				continue;
// 			}
// 			break;
// 		}
		double Seperate, Middle, LastMiddle, m_MinRange = 0, m_MaxRange = m_TempTargetList[n].m_TargetPosO.rt;
		Seperate = Middle = LastMiddle = m_MaxRange;
		while (Seperate > m_TempTargetList[n].m_RangeResolution)
		{
			m_TempTargetList[n].m_TargetPosO.rt = Middle;
			m_ObsvCoord = POLAROBSV(m_TempTargetList[n].m_TargetPosO);
			m_TempTargetList[n].m_TargetPosG = OBSV2GEOG(m_ObsvCoord, m_ElecReconPos);

			R2 = 4120 * (pow(m_TempTargetList[n].m_TargetPosG.ht, 0.5) + pow(m_ElecReconPos.ht, 0.5));
			if (m_TempTargetList[n].m_TargetPosG.ht < 0 || R2 < Middle || !InterVisibility(m_ElecReconPos, m_TempTargetList[n].m_TargetPosG))
			{
				LastMiddle = Middle;
				m_MaxRange = Middle;
				Middle = 0.5*(m_MaxRange + m_MinRange);
				Seperate = fabs(LastMiddle - Middle);
			}
			else
			{
				m_MinRange = Middle;
				LastMiddle = Middle;
				Middle = 0.5*(m_MinRange + m_MaxRange);
				Seperate = fabs(LastMiddle - Middle);
			}
		}


		m_MaxRange = m_TempTargetList[n].m_TargetPosO.rt;
		Seperate = Middle = LastMiddle = m_MaxRange;
		m_MinRange = 0;
		while (Seperate > m_TempTargetList[n].m_RangeResolution)
		{
			m_TempTargetList[n].m_TargetPosO.rt = Middle;
			m_ObsvCoord = POLAROBSV(m_TempTargetList[n].m_TargetPosO);
			m_TempTargetList[n].m_TargetPosG = OBSV2GEOG(m_ObsvCoord, m_ElecReconPos);

			//////////////�Զ�������ջ�ָ��
			m_ElecReconEquipList[0].m_AnteMBAz = m_TempTargetList[n].m_TargetPosO.az*r2d;
			m_ElecReconEquipList[0].m_AnteMBEl = m_TempTargetList[n].m_TargetPosO.el*r2d;
			ChangeAzFrom360To180(m_ElecReconEquipList[0].m_AnteMBAz, false);
			PreJamPowerCal();

			double m_TempEl = m_ElecReconEquipList[0].m_AnteMBEl*d2r;
			double m_MultipathCoef = CalMultiPathCoef2(m_ElecReconPos, m_TempTargetList[n].m_TargetPosG, m_AntennaPara, m_TempEl, m_ElecReconAtt, m_RadiatePara.m_RadioFreqency);

			//��ȡ��Ӧ�ĸ��Ų���
			JamParam.TransPower = m_RadiatePara.m_TransPower;
			JamParam.TransGain = m_RadiatePara.m_AnteGain;
			JamParam.TransLoss = m_RadiatePara.m_RadiateLoss;//������ͬ������Ƶ�������
			JamParam.RadarWavelength = LIGHTSPEED / m_RadiatePara.m_RadioFreqency;
			JamParam.JamRange = m_TempTargetList[n].m_TargetPosO.rt;
			JamParam.ReceiveLoss = m_ElecReconEquipList[0].m_IntegratedRevLoss;
			//JamParam.Attenuation = 1.8;//���������Ĭ�ϴ����������ֵ
			JamParam.Attenuation = GetAllBroadCastLostDB(m_WeatherPara, m_TempTargetList[n].m_TargetPosG, m_ElecReconPos, m_RadiatePara.m_RadioFreqency);
			JamParam.ReceiveGain = m_ElecReconEquipList[0].m_AnteGain;
			JamParam.BandRatioFactor = m_FC;
			EchoPower = ReceiveJamPowerCalculate(JamParam);
			EchoPower *= m_MultipathCoef;
			//�����Ÿɱ�

			SNR = 10.0*log10(EchoPower / (m_ElecReconEquipList[0].m_NoisePower + m_ElecReconEquipList[0].m_JamPower + EnvironmentNoisePower) + eps);
			if (SNR > 14)//��Ӧ�����ʴ���50%
			{
				m_MinRange = Middle;
				LastMiddle = Middle;
				Middle = 0.5*(m_MinRange + m_MaxRange);
				Seperate = fabs(LastMiddle - Middle);
			}
			else
			{
				LastMiddle = Middle;
				m_MaxRange = Middle;
				Middle = 0.5*(m_MinRange + m_MaxRange);
				Seperate = fabs(LastMiddle - Middle);
			}
		}


// 		for (R1 = m_TempTargetList[n].m_TargetPosO.rt; R1 > 0; R1 -= m_TempTargetList[n].m_RangeResolution)
// 		{
// 			m_TempTargetList[n].m_TargetPosO.rt = R1;
// 			m_ObsvCoord = POLAROBSV(m_TempTargetList[n].m_TargetPosO);
// 			m_TempTargetList[n].m_TargetPosG = OBSV2GEOG(m_ObsvCoord, m_ElecReconPos);
// 			//��ȡ��Ӧ�ĸ��Ų���
// 			JamParam.TransPower = m_RadiatePara.m_TransPower;
// 			JamParam.TransGain = m_RadiatePara.m_AnteGain;
// 			JamParam.TransLoss = m_RadiatePara.m_RadiateLoss;//������ͬ������Ƶ�������
// 			JamParam.RadarWavelength = LIGHTSPEED / m_RadiatePara.m_RadioFreqency;
// 			JamParam.JamRange = m_TempTargetList[n].m_TargetPosO.rt;
// 			JamParam.ReceiveLoss = m_ElecReconEquipList[0].m_IntegratedRevLoss;
// 			//JamParam.Attenuation = 1.8;//���������Ĭ�ϴ����������ֵ
// 			JamParam.Attenuation = GetAllBroadCastLostDB(m_WeatherPara, m_TempTargetList[n].m_TargetPosG, m_ElecReconPos, m_RadiatePara.m_RadioFreqency);
// 			JamParam.ReceiveGain = m_ElecReconEquipList[0].m_AnteGain;
// 			JamParam.BandRatioFactor = m_FC;
// 			EchoPower = ReceiveJamPowerCalculate(JamParam);
// 			//�����Ÿɱ�
// 			SNR = 10.0*log10(EchoPower / (m_ElecReconEquipList[0].m_NoisePower + m_ElecReconEquipList[0].m_JamPower) + eps);
// 			if (SNR > 14)//��Ӧ�����ʴ���50%
// 			{
// 				break;
// 			}
// 		}
	}
}
bool CElecReconProcess::ThresholdDetector(double Pd)
{
	//[ע��]:��δ����ڿ�ƽ̨����ʱ���ܻ��������
	double r = (double)rand() / (RAND_MAX + 1);

	return r <= Pd ? true : false;
}
//���������
double CElecReconProcess::AngleDiff(double SNR, double BeamWidth)
{
	double result = 0.51*BeamWidth / sqrt(pow(10, SNR / 10.0));
	result = result*GaussianGeneration();
	//��ʱɾ��
	//while (fabs(result)>BeamWidth/10.0) result=result/2.0;
	result *= (PI / 180);//
	//printf("��%d��Ŀ��ķ�λ�Ƕ����Ϊ��%f\n", theIndex, result);
	return result;
}
void CElecReconProcess::SimpleBeamAdvance()
{
	long m_TargetNum = m_RadiateList.size();
	//ÿһ��̽�⣬�״������������
	long m_TrackID = 1;
	short i(0);
#pragma omp parallel for
	for (i = 0; i < m_TargetNum; i++)
	{
		bool AzFlag(false), ElFlag(false), RtFlag(false);
		double EchoPower(0.0), SNR(0.0);
		double R2(0.0), DetRange(0.0);
		double NBJPower(0.0), MFTPower(0.0);//�ֱ�Ϊ�������Ź��ʡ���Ŀ����Ź���
		double JamTxGain(0.0), JamRxGain(0.0);//���ŷ��䡢��������(��λ��ΪdB)
		double IFactor(0.0);//��Ƶ�ݱ��������(������)
		//�ɸ��Ų�������ٵ㼣��Ӧ�Ĳ���ֵ
		double JRt(0.0), JAz(0.0), JEl(0.0);



		BEAMSTATEPARA CurBeam;
		GEOG_COORD tar, j_tar;
		OBSV_COORD radar;
		MSLB_COORD Missile;
		MSLB_POLAR polar, TarPosPolar, FTPolar;
		EULER_ANGLE pose;
		TARGETREALPOS RealPos;
		PowerCalculateParam PCParam;
		PASSIVEDETECTRADIATION PassiveDetObject;
		CFAgilityParam CFAParam;
		JamPowerCalculateParam JamParam;
		ECEF_COORD  TargetPosE;



		//ȡ����ǰĿ���Ӧ��λ��
		memset(&tar, 0, sizeof(GEOG_COORD));
		tar.ln = m_RadiateList[i].m_TargetPara.m_TargetLon*d2r;
		tar.lt = m_RadiateList[i].m_TargetPara.m_TargetLat*d2r;
		tar.ht = m_RadiateList[i].m_TargetPara.m_TargetAlt;

		radar = GEOG2OBSV(tar, m_ElecReconPos);
		Missile = OBSV2MSLB(radar, m_ElecReconAtt);
		polar = MSLBPOLAR(Missile);

		//			if (polar.az < 0.0)	polar.az += 2.0 * PI;

		R2 = 4120 * (pow(m_RadiateList[i].m_TargetPara.m_TargetAlt, 0.5) + pow(m_ElecReconPos.ht, 0.5));
		//			DetRange = R1 < R2 ? R1 : R2;
		DetRange = R2;

		if (polar.rt > DetRange)
		{
			continue;
		}
		//�ཻ��ϵ�о�
		AzFlag = ComputeAzIntersectionAngle(polar.az, m_AnteMBAz, true) <= m_PassiveRadarEquip.m_AzBeamWidth*d2r*0.50;
		ElFlag = fabs(polar.el - m_AnteMBEl) <= m_PassiveRadarEquip.m_ElBeamWidth*d2r*0.50;

		if (!AzFlag || !ElFlag)
		{
			continue;
		}

		EULER_ANGLE			m_RadiateComAtt;				//ʵʱ��̬��
		double				m_Gain;
		m_RadiateComAtt.roll = m_RadiateList[i].m_TargetPara.m_TargetPhy *d2r;//��ת��
		m_RadiateComAtt.yaw = -m_RadiateList[i].m_TargetPara.m_TargetPsi *d2r;  //ƫ����	
		m_RadiateComAtt.pitch = m_RadiateList[i].m_TargetPara.m_TargetTheta *d2r;//������

		radar = GEOG2OBSV(m_ElecReconPos,tar);
		Missile = OBSV2MSLB(radar, m_RadiateComAtt);
		TarPosPolar = MSLBPOLAR(Missile);

		//�ཻ��ϵ�о�
		AzFlag = ComputeAzIntersectionAngle(TarPosPolar.az, -m_RadiateList[i].m_RadiatePara.m_AnteMBAz*d2r, true) <= m_RadiateList[i].m_RadiatePara.m_AzBeamWidth*d2r*0.50;
		ElFlag = fabs(TarPosPolar.el - m_RadiateList[i].m_RadiatePara.m_AnteMBEl*d2r) <= m_RadiateList[i].m_RadiatePara.m_ElBeamWidth*d2r*0.50;

		if (!AzFlag || !ElFlag)
		{
			m_Gain = m_RadiateList[i].m_RadiatePara.m_AnteGain - 35;
		}
		else
		{
			m_Gain = m_RadiateList[i].m_RadiatePara.m_AnteGain;
		}

		double m_FC = FrequencyCover(m_PassiveRadarEquip.m_BandWidthDownLimit, m_PassiveRadarEquip.m_BandWidthUpLimit, m_RadiateList[i].m_RadiatePara.m_RadioFreqency, m_RadiateList[i].m_RadiatePara.m_BandWidth);
		if (m_FC < eps)
		{
			continue;
		}
		if (!InterVisibility(m_ElecReconPos, tar))
		{
			continue;
		}
		m_RadiateList[i].beProcessed = true;
		/////////////////////////����ÿһ���״���Ŀ�괦�ķ���ǿ��
		JamParam.TransPower = m_RadiateList[i].m_RadiatePara.m_TransPower;
		JamParam.TransGain = m_Gain;
		JamParam.TransLoss = m_RadiateList[i].m_RadiatePara.m_RadiateLoss;//������ͬ������Ƶ�������
		JamParam.RadarWavelength = LIGHTSPEED / m_RadiateList[i].m_RadiatePara.m_RadioFreqency;
		JamParam.JamRange = polar.rt;
		JamParam.ReceiveLoss = m_PassiveRadarEquip.m_IntegratedRevLoss;
		JamParam.Attenuation = GetAllBroadCastLostDB(m_WeatherPara, tar, m_ElecReconPos, m_RadiateList[i].m_RadiatePara.m_RadioFreqency);
		JamParam.ReceiveGain = m_PassiveRadarEquip.m_AnteGain;
		JamParam.BandRatioFactor = m_FC;

		//���㵽���״���ջ�ǰ�˵���������ֵ���ú�����REWS���ܿ���ʵ��
		EchoPower = ReceiveJamPowerCalculate(JamParam);
		//���������
		SNR = 10.0*log10(EchoPower / (m_PassiveRadarEquip.m_NoisePower + eps));
		double Pd = DetectProbability(FIXRCS, SNR, 1.0e-6);
		if (ThresholdDetector(Pd))
		{
			//�Ѿ����Ŀ�����о�
			strcpy_s(PassiveDetObject.m_TargetID, m_RadiateList[i].m_TargetPara.m_TargetID);
			PassiveDetObject.Az = polar.az;
			PassiveDetObject.El = polar.el;
			PassiveDetObject.m_AzAngleDiff = AngleDiff(SNR, m_PassiveRadarEquip.m_AzBeamWidth);
			PassiveDetObject.m_ElAngleDiff = AngleDiff(SNR, m_PassiveRadarEquip.m_ElBeamWidth);
			PassiveDetObject.SNR = SNR;
			PassiveDetObject.m_TargetNum = 1;
			omp_set_lock(&Passivelock);
			PassiveDetObject.m_TrackID = m_TrackID++;
			m_PassiveDetectList.push_back(PassiveDetObject);
			omp_unset_lock(&Passivelock);
			//break;
		}
	}
}
void CElecReconProcess::BeamAdvance()
{
	long BeamNum = m_Beams.size();
	long m_TargetNum = m_RadiateList.size();
	//������ִ������ѭ��:�ռ䲨λ������Ŀ��
#pragma omp parallel for
	for (long n = 0; n < BeamNum; n++)
	{
		short i(0);
		bool AzFlag(false), ElFlag(false), RtFlag(false);
		double EchoPower(0.0), SNR(0.0);
		double R2(0.0), DetRange(0.0);
		double NBJPower(0.0), MFTPower(0.0);//�ֱ�Ϊ�������Ź��ʡ���Ŀ����Ź���
		double JamTxGain(0.0), JamRxGain(0.0);//���ŷ��䡢��������(��λ��ΪdB)
		double IFactor(0.0);//��Ƶ�ݱ��������(������)
		//�ɸ��Ų�������ٵ㼣��Ӧ�Ĳ���ֵ
		double JRt(0.0), JAz(0.0), JEl(0.0);


		double ReceivePowerAccumulate, ElAccumulate, AzAccumulate;
		long TargetNum;
		//ÿһ��̽�⣬�״������������
		long m_TrackID = 1;

		BEAMSTATEPARA CurBeam;
		GEOG_COORD tar, j_tar;
		OBSV_COORD radar;
		MSLB_COORD Missile;
		MSLB_POLAR polar, TarPosPolar, FTPolar;
		EULER_ANGLE pose;
		TARGETREALPOS RealPos;
		PowerCalculateParam PCParam;
		PASSIVEDETECTRADIATION PassiveDetObject;
		CFAgilityParam CFAParam;
		JamPowerCalculateParam JamParam;
		ECEF_COORD  TargetPosE;

		//ȡ����ǰ���䲨��(�������䷽λ�Ͳ���פ��ʱ��)
		CurBeam = m_Beams[n];

		///����غ��Ӳ�
//		GroundSeaNoisePower = GetGroundSeaNoisePowerBeam(CurBeam);
		double m_BeamAntMBEl = CurBeam.m_AnteMBEl*d2r;

		ReceivePowerAccumulate = 0;
		ElAccumulate = 0;
		AzAccumulate = 0;
		TargetNum = 0;
		double MaxPower = -1;
		char MaxPowerTargetID[256];
		for (i = 0; i < m_TargetNum; i++)
		{
			if (m_RadiateList[i].beProcessed)
			{
				continue;
			}
			//ȡ����ǰĿ���Ӧ��λ��
			memset(&tar, 0, sizeof(GEOG_COORD));
			tar.ln = m_RadiateList[i].m_TargetPara.m_TargetLon*d2r;
			tar.lt = m_RadiateList[i].m_TargetPara.m_TargetLat*d2r;
			tar.ht = m_RadiateList[i].m_TargetPara.m_TargetAlt;

			radar = GEOG2OBSV(tar, m_ElecReconPos);
			Missile = OBSV2MSLB(radar, m_ElecReconAtt);
			polar = MSLBPOLAR(Missile);

//			if (polar.az < 0.0)	polar.az += 2.0 * PI;

			R2 = 4120 * (pow(m_RadiateList[i].m_TargetPara.m_TargetAlt, 0.5) + pow(m_ElecReconPos.ht, 0.5));
			//			DetRange = R1 < R2 ? R1 : R2;
			DetRange = R2;

			if (polar.rt > DetRange)
			{
				continue;
			}
			//�ཻ��ϵ�о�
			AzFlag = ComputeAzIntersectionAngle(polar.az, CurBeam.m_AnteMBAz*d2r, true) <= m_PassiveRadarEquip.m_AzBeamWidth*d2r*0.50;
			ElFlag = fabs(polar.el - CurBeam.m_AnteMBEl*d2r) <= m_PassiveRadarEquip.m_ElBeamWidth*d2r*0.50;

			if (!AzFlag || !ElFlag)
			{
				continue;
			}
			double m_FC = FrequencyCover(m_PassiveRadarEquip.m_BandWidthDownLimit, m_PassiveRadarEquip.m_BandWidthUpLimit, m_RadiateList[i].m_RadiatePara.m_RadioFreqency, m_RadiateList[i].m_RadiatePara.m_BandWidth);
			if (m_FC < eps)
			{
				continue;
			}

			EULER_ANGLE			m_RadiateComAtt;				//ʵʱ��̬��
			double				m_Gain;
			m_RadiateComAtt.roll = m_RadiateList[i].m_TargetPara.m_TargetPhy *d2r;//��ת��
			m_RadiateComAtt.yaw = -m_RadiateList[i].m_TargetPara.m_TargetPsi *d2r;  //ƫ����	
			m_RadiateComAtt.pitch = m_RadiateList[i].m_TargetPara.m_TargetTheta *d2r;//������

			radar = GEOG2OBSV(m_ElecReconPos, tar);
			Missile = OBSV2MSLB(radar, m_RadiateComAtt);
			TarPosPolar = MSLBPOLAR(Missile);

			//�ཻ��ϵ�о�
			AzFlag = ComputeAzIntersectionAngle(TarPosPolar.az, -m_RadiateList[i].m_RadiatePara.m_AnteMBAz*d2r, true) <= m_RadiateList[i].m_RadiatePara.m_AzBeamWidth*d2r*0.50;
			ElFlag = fabs(TarPosPolar.el - m_RadiateList[i].m_RadiatePara.m_AnteMBEl*d2r) <= m_RadiateList[i].m_RadiatePara.m_ElBeamWidth*d2r*0.50;

			if (!AzFlag || !ElFlag)
			{
				m_Gain = m_RadiateList[i].m_RadiatePara.m_AnteGain - 35;
			}
			else
			{
				m_Gain = m_RadiateList[i].m_RadiatePara.m_AnteGain;
			}

			if (!InterVisibility(m_ElecReconPos, tar))
			{
				continue;
			}
			m_RadiateList[i].beProcessed = true;
			/////////////////////////����ÿһ���״���Ŀ�괦�ķ���ǿ��
			JamParam.TransPower = m_RadiateList[i].m_RadiatePara.m_TransPower;
			JamParam.TransGain = m_Gain;
			JamParam.TransLoss = m_RadiateList[i].m_RadiatePara.m_RadiateLoss;//������ͬ������Ƶ�������
			JamParam.RadarWavelength = LIGHTSPEED / m_RadiateList[i].m_RadiatePara.m_RadioFreqency;
			JamParam.JamRange = polar.rt;
			JamParam.ReceiveLoss = m_PassiveRadarEquip.m_IntegratedRevLoss;
			JamParam.Attenuation = GetAllBroadCastLostDB(m_WeatherPara, tar, m_ElecReconPos, m_RadiateList[i].m_RadiatePara.m_RadioFreqency);
			JamParam.ReceiveGain = m_PassiveRadarEquip.m_AnteGain;
			JamParam.BandRatioFactor = m_FC;

			//���㵽���״���ջ�ǰ�˵���������ֵ���ú�����REWS���ܿ���ʵ��
			EchoPower = ReceiveJamPowerCalculate(JamParam);

			if (MaxPower<0 || MaxPower<EchoPower)
			{
				strcpy_s(MaxPowerTargetID, m_RadiateList[i].m_TargetPara.m_TargetID);
			}

			ReceivePowerAccumulate += EchoPower;
			ElAccumulate = ElAccumulate + polar.el*EchoPower;
			AzAccumulate = AzAccumulate + polar.az*EchoPower;
			TargetNum++;
		}
		if (ReceivePowerAccumulate< eps)
		{
			continue;
		}
		//���������
		SNR = 10.0*log10(ReceivePowerAccumulate / (m_PassiveRadarEquip.m_NoisePower + eps));
		double Pd = DetectProbability(FIXRCS, SNR, 1.0e-6); 
		if (ThresholdDetector(Pd))
		{
			//�Ѿ����Ŀ�����о�
			strcpy_s(PassiveDetObject.m_TargetID,MaxPowerTargetID);
			PassiveDetObject.Az = AzAccumulate / ReceivePowerAccumulate;
			PassiveDetObject.El = ElAccumulate / ReceivePowerAccumulate;
			PassiveDetObject.m_AzAngleDiff = AngleDiff(SNR, m_PassiveRadarEquip.m_AzBeamWidth);
			PassiveDetObject.m_ElAngleDiff = AngleDiff(SNR, m_PassiveRadarEquip.m_ElBeamWidth);
			PassiveDetObject.SNR = SNR;
			PassiveDetObject.m_TargetNum = TargetNum;
			omp_set_lock(&Passivelock);
			PassiveDetObject.m_TrackID = m_TrackID++;
			//��ɸ�ֵ���������б���
			m_PassiveDetectList.push_back(PassiveDetObject);
			omp_unset_lock(&Passivelock);
			//�Ѿ��ɹ����ҵ��ò����ڵ�Ŀ�꣬�����Ŀ�����ѭ��
			//break;
		}
	}
}
void CElecReconProcess::Advance()
{
	PreJamPowerCal();

	GEOG_COORD tar;
	OBSV_COORD radar;
	MSLB_COORD plane;
	MSLB_POLAR polar;
	EULER_ANGLE m_RadarAtt,pose;
	JamPowerCalculateParam JamParam;
	long n,m;
	bool AzFlag, ElFlag;
	double R2;
	double  RadiateGain(0.0);
	double EchoPower(0.0), SNR(0.0);
	FINDRADIATION m_FindRadiation;
	m_FindRadiationList.clear();
	for (n = 0; n < m_RadarList.size();n++)
	{
		tar.lt = m_RadarList[n].m_PLATFORMPARA.m_Lat*d2r;
		tar.ln = m_RadarList[n].m_PLATFORMPARA.m_Lon*d2r;
		tar.ht = m_RadarList[n].m_PLATFORMPARA.m_Alt;
		
		m_RadarAtt.roll = m_RadarList[n].m_PLATFORMPARA.m_Phy *d2r;//��ת��
		m_RadarAtt.yaw = m_RadarList[n].m_PLATFORMPARA.m_Psi *d2r;  //ƫ����	
		m_RadarAtt.pitch = m_RadarList[n].m_PLATFORMPARA.m_Theta *d2r;//������

		radar = GEOG2OBSV(m_ElecReconPos, tar);
		plane = OBSV2MSLB(radar, m_RadarAtt);
		polar = MSLBPOLAR(plane);
		//if (polar.az < 0.0)	polar.az += 2.0 * PI;

		R2 = 4120 * (pow(m_ElecReconPos.ht, 0.5) + pow(tar.ht, 0.5));
		if (R2<polar.rt)
		{
			continue;
		}
///////////////////////////////////////////////////////////////////////////////
		AzFlag = fabs(polar.az) <= m_RadarList[n].m_RadarEquipPara.m_AzScanAngle* d2r*0.50;
		ElFlag = fabs(polar.el) <= m_RadarList[n].m_RadarEquipPara.m_ElScanAngle* d2r*0.50;
		RadiateGain = (AzFlag && ElFlag) ? m_RadarList[n].m_RadarAntennaPara.m_AnteMBGain : m_RadarList[n].m_RadarAntennaPara.m_AnteSBGain;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		radar = GEOG2OBSV(tar, m_ElecReconPos);
		plane = OBSV2MSLB(radar, m_ElecReconAtt);
		polar = MSLBPOLAR(plane);
		//if (polar.az < 0.0)	polar.az += 2.0 * PI;

		if (!InterVisibility(m_ElecReconPos, tar))
		{
			continue;
		}
		for (m = 0; m < m_ElecReconEquipNum;m++)
		{
			AzFlag = ComputeAzIntersectionAngle(polar.az, m_ElecReconEquipList[m].m_AnteMBAz*d2r, true) <= m_ElecReconEquipList[m].m_AzBeamWidth* d2r*0.50;
			ElFlag = fabs(polar.el - m_ElecReconEquipList[m].m_AnteMBEl* d2r) <= m_ElecReconEquipList[m].m_ElBeamWidth* d2r*0.50;
			if (!AzFlag ||! ElFlag)
				continue;
			//double m_Attenuation = 1.8;//���������Ĭ�ϴ����������ֵ
			double m_FC = FrequencyCover(m_ElecReconEquipList[m].m_BandWidthDownLimit, m_ElecReconEquipList[m].m_BandWidthUpLimit, m_RadarList[n].m_RadarEquipPara.m_RadioFreqency, m_RadarList[n].m_RadarEquipPara.m_BandWidth);
			if (m_FC < eps)
				continue;
			double m_TempEl = m_ElecReconEquipList[m].m_AnteMBEl*d2r;
			double m_MultipathCoef = CalMultiPathCoef2(m_ElecReconPos, tar, m_AntennaPara, m_TempEl, m_ElecReconAtt, m_RadarList[n].m_RadarEquipPara.m_RadioFreqency);
			//��ȡ��Ӧ�ĸ��Ų���
			JamParam.TransPower = m_RadarList[n].m_RadarEquipPara.m_TransPower;
			JamParam.TransGain = RadiateGain;
			JamParam.TransLoss = m_RadarList[n].m_RadarEquipPara.m_RadiateLoss;//������ͬ������Ƶ�������
			JamParam.RadarWavelength = LIGHTSPEED / m_RadarList[n].m_RadarEquipPara.m_RadioFreqency;
			JamParam.JamRange = polar.rt;
			JamParam.ReceiveLoss = m_ElecReconEquipList[m].m_IntegratedRevLoss;
			//JamParam.Attenuation = 1.8;//���������Ĭ�ϴ����������ֵ
			JamParam.Attenuation = GetAllBroadCastLostDB(m_WeatherPara, tar, m_ElecReconPos, m_RadarList[n].m_RadarEquipPara.m_RadioFreqency);
			JamParam.ReceiveGain = m_ElecReconEquipList[m].m_AnteGain;
			JamParam.BandRatioFactor = m_FC/*1.2*CFAParam.FMBandwidth / CFAParam.JamBandwidth;*/;
			EchoPower = ReceiveJamPowerCalculate(JamParam);
			//EchoPower = m_FC*m_RadarList[n].m_RadarEquipPara.m_TransPower*pow(10.0, (RadiateGain + ReconGain - m_RadarList[n].m_RadarEquipPara.m_RadiateLoss - m_ElecReconEquipList[m].m_IntegratedRevLoss - m_Attenuation) / 10.0);
			//EchoPower = EchoPower / ((m_RadarList[n].m_RadarEquipPara.m_RadioFreqency * 4 * PI*polar.rt)*(m_RadarList[n].m_RadarEquipPara.m_RadioFreqency * 4 * PI*polar.rt));
			if (EchoPower<eps)
			{
				continue;
			}
			EchoPower*=m_MultipathCoef;
			//�����Ÿɱ�
			SNR = 10.0*log10(EchoPower / (m_ElecReconEquipList[m].m_NoisePower + m_ElecReconEquipList[m].m_JamPower) + eps);

			if (SNR>=14)
			{
				m_FindRadiation.IsRadar = 1;
				strcpy_s(m_FindRadiation.m_RadarID,m_RadarList[n].m_RadarID);
				strcpy_s(m_FindRadiation.m_PlatformID, m_RadarList[n].m_PLATFORMPARA.m_PlatformID);
				strcpy_s(m_FindRadiation.m_ComID, "\0");
				m_FindRadiationList.push_back(m_FindRadiation);
				break;
			}
		}
	}


	for (n = 0; n < m_ComList.size(); n++)
	{
		tar.lt = m_ComList[n].m_ElecReconPlatform.m_Lat*d2r;
		tar.ln = m_ComList[n].m_ElecReconPlatform.m_Lon*d2r;
		tar.ht = m_ComList[n].m_ElecReconPlatform.m_Alt;

		m_RadarAtt.roll = m_ComList[n].m_ElecReconPlatform.m_Phy *d2r;//��ת��
		m_RadarAtt.yaw = m_ComList[n].m_ElecReconPlatform.m_Psi *d2r;  //ƫ����	
		m_RadarAtt.pitch = m_ComList[n].m_ElecReconPlatform.m_Theta *d2r;//������

		radar = GEOG2OBSV(m_ElecReconPos, tar);
		plane = OBSV2MSLB(radar, m_RadarAtt);
		polar = MSLBPOLAR(plane);
		//if (polar.az < 0.0)	polar.az += 2.0 * PI;

		R2 = 4120 * (pow(m_ElecReconPos.ht, 0.5) + pow(tar.ht, 0.5));
		if (R2 < polar.rt)
		{
			continue;
		}
		///////////////////////////////////////////////////////////////////////////////
		AzFlag = ComputeAzIntersectionAngle(polar.az, m_ComList[n].m_ComEquip.m_AnteMBAz, true) <= m_ComList[n].m_ComEquip.m_AzBeamWidth*0.50* d2r;
		ElFlag = fabs(polar.el - m_ComList[n].m_ComEquip.m_AnteMBEl) <= m_ComList[n].m_ComEquip.m_ElBeamWidth*0.50* d2r;
		
		if (!AzFlag || !ElFlag)
		{
			continue;
		}
		RadiateGain = m_ComList[n].m_ComEquip.m_AnteGain;

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		radar = GEOG2OBSV(tar, m_ElecReconPos);
		plane = OBSV2MSLB(radar, m_ElecReconAtt);
		polar = MSLBPOLAR(plane);
		//if (polar.az < 0.0)	polar.az += 2.0 * PI;

		if (!InterVisibility(m_ElecReconPos, tar))
		{
			continue;
		}

		for (m = 0; m < m_ElecReconEquipNum; m++)
		{
			AzFlag = ComputeAzIntersectionAngle(polar.az, m_ElecReconEquipList[m].m_AnteMBAz*d2r, true) <= m_ElecReconEquipList[m].m_AzBeamWidth* d2r*0.50;
			ElFlag = fabs(polar.el - m_ElecReconEquipList[m].m_AnteMBEl* d2r) <= m_ElecReconEquipList[m].m_ElBeamWidth* d2r*0.50;
			if (!AzFlag || !ElFlag)
				continue;
			//double m_Attenuation = 1.8;//���������Ĭ�ϴ����������ֵ
			double m_FC = FrequencyCover(m_ElecReconEquipList[m].m_BandWidthDownLimit, m_ElecReconEquipList[m].m_BandWidthUpLimit, m_ComList[n].m_ComEquip.m_RadioFreqency, m_ComList[n].m_ComEquip.m_BandWidth);
			
			if (m_FC < eps)
				continue;
			double m_TempEl = m_ElecReconEquipList[m].m_AnteMBEl*d2r;
			double m_MultipathCoef = CalMultiPathCoef2(m_ElecReconPos, tar, m_AntennaPara, m_TempEl, m_ElecReconAtt, m_ComList[n].m_ComEquip.m_RadioFreqency);
			//��ȡ��Ӧ�ĸ��Ų���
			JamParam.TransPower = m_ComList[n].m_ComEquip.m_TransPower;
			JamParam.TransGain = RadiateGain;
			JamParam.TransLoss = m_ComList[n].m_ComEquip.m_RadiateLoss;//������ͬ������Ƶ�������
			JamParam.RadarWavelength = LIGHTSPEED / m_ComList[n].m_ComEquip.m_RadioFreqency;
			JamParam.JamRange = polar.rt;
			JamParam.ReceiveLoss = m_ElecReconEquipList[m].m_IntegratedRevLoss;
			//JamParam.Attenuation = 1.8;//���������Ĭ�ϴ����������ֵ
			JamParam.Attenuation = GetAllBroadCastLostDB(m_WeatherPara, tar, m_ElecReconPos, m_ComList[n].m_ComEquip.m_RadioFreqency);
			JamParam.ReceiveGain = m_ElecReconEquipList[m].m_AnteGain;
			JamParam.BandRatioFactor = m_FC/*1.2*CFAParam.FMBandwidth / CFAParam.JamBandwidth;*/;
			EchoPower = ReceiveJamPowerCalculate(JamParam);
			//EchoPower = m_FC*m_RadarList[n].m_RadarEquipPara.m_TransPower*pow(10.0, (RadiateGain + ReconGain - m_RadarList[n].m_RadarEquipPara.m_RadiateLoss - m_ElecReconEquipList[m].m_IntegratedRevLoss - m_Attenuation) / 10.0);
			//EchoPower = EchoPower / ((m_RadarList[n].m_RadarEquipPara.m_RadioFreqency * 4 * PI*polar.rt)*(m_RadarList[n].m_RadarEquipPara.m_RadioFreqency * 4 * PI*polar.rt));
			EchoPower *= m_MultipathCoef;
			if (EchoPower < eps)
			{
				continue;
			}
			//�����Ÿɱ�
			SNR = 10.0*log10(EchoPower / (m_ElecReconEquipList[m].m_NoisePower + m_ElecReconEquipList[m].m_JamPower) + eps);

			if (SNR >= 14)
			{
				m_FindRadiation.IsRadar = 0;
				strcpy_s(m_FindRadiation.m_RadarID, "\0");
				strcpy_s(m_FindRadiation.m_PlatformID, m_ComList[n].m_ElecReconPlatform.m_PlatformID);
				strcpy_s(m_FindRadiation.m_ComID, m_ComList[n].m_ComID);
				m_FindRadiationList.push_back(m_FindRadiation);
				break;
			}
		}
	}

}