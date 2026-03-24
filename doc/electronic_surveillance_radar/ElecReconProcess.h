#ifndef ElecReconProcess_h__
#define ElecReconProcess_h__

#include "model_typedef.h"
#include "Coordinates/CoordDefinitions.h"
#include "Base/Constants.h"
#include <vector>
#include "omp.h"


#pragma once
class CElecReconProcess
{
public:
	CElecReconProcess();
	~CElecReconProcess();

	//传入场景中包含的各类干扰机(压制式、欺骗式)
	void SetJammers(std::vector<JAMMARPARA>& jam);

	//设置自然环境参数
	void SetWeather(const WEATHERPARA& data) { m_WeatherPara = data; }

	//设置地理环境参数
	void SetGeoData(const GEOPARA& data) { m_GeoPara = data; }

	bool Initialize(const ELECRECONPARA&  ElecReconPara);
	bool Initialize(PASSIVERADARPARA& PassiveRadarPara,
		std::vector<RADIATEDETECTPARA>&	RadiateList,
		std::vector<RADIATEDETECTPARA>&	GuideRadarList,
		std::vector<TARGETPARA>&        TargetList);
	bool Initialize(const ELECRECONEQUIPPARA& m_ElecReconPara, const RADIATEPARA& m_RadiatePara, const RADIATEPLATFORMWORKPARA3D& m_RadiatePlatformWorkPara3D);
	bool Initialize(const ELECRECONEQUIPPARA& ElecReconPara, const RADIATEPARA& RadiatePara, const  RADIATEPLATFORMWORKPARA2D2& RadiatePlatformWorkPara2D2);
	bool Initialize(const ELECRECONEQUIPPARA& ElecReconPara, const RADIATEPARA& RadiatePara, const  RADIATEPLATFORMWORKPARA2D& RadiatePlatformWorkPara2D);

	void PreJamPowerCal();

	void Advance();

	long TempTargetArrange(const RADIATEPLATFORMWORKPARA3D m_RadiatePlatformWorkPara3D);
	bool ThresholdDetector(double Pd);
	void ScopeAdvance3D();
	void ScopeAdvance2D2();
	void ScopeAdvance2D();
	//计算测角误差
	double AngleDiff(double SNR, double BeamWidth);
	void SetRadars(std::vector<RADARDETECTPARA>& Radar);

	void SetComs(std::vector<COMPARA>& Com);

	long TempTargetArrange(const RADIATEPLATFORMWORKPARA2D& RadiatePlatformWorkPara2D);

	long TempTargetArrange(const RADIATEPLATFORMWORKPARA2D2 m_RadiatePlatformWorkPara2D2);
	void SimpleBeamAdvance();
	void BeamAdvance();
	long BeamArrange(std::vector<BEAMSTATEPARA>& Beams);
	std::vector<FINDRADIATION>       m_FindRadiationList;
	long NumAZ, NumEL;
	std::vector<TEMPTARGETPARA>	     m_TempTargetList;	//本次探测对应的所有目标列表
	std::vector<PASSIVEDETECTRADIATION>       m_PassiveDetectList;
	////////用于被动雷达制导
	short       m_SimplePassiveMode;    //简单被动探测
	double      m_SimTime;
private:
	// 根据工作频率确定波段类型
	short JudgeWaveType(double _RadioFreqency);

protected:
	WEATHERPARA			m_WeatherPara;			//自然环境相关参数
	GEOPARA				m_GeoPara;				//地理环境(地表和海面)参数

	PLATFORMPARA        m_ElecReconPlatform;
	short               m_ElecReconEquipNum;
	ELECRECONEQUIPPARA  m_ElecReconEquipList[10];
	std::vector<RADARDETECTPARA>  m_RadarList;
	std::vector< COMPARA >        m_ComList;
	std::vector<JAMMARPARA>       m_JammarList;


	GEOG_COORD			m_ElecReconPos;				//当前电子侦察机部署平台实时位置(地理系)
	EULER_ANGLE			m_ElecReconAtt;				//当前电子侦察机部署平台实时姿态角
	RADIATEPARA         m_RadiatePara;
///用于雷达探测计算
	RADARANTENNAPARA	m_AntennaPara;
	ELECRECONEQUIPPARA  m_PassiveRadarEquip;
	std::vector<BEAMSTATEPARA>	m_Beams;		//本次探测安排的所有照射波位

	std::vector<RADIATEDETECTPARA>	m_RadiateList;
	omp_lock_t Passivelock;
	////////用于被动雷达制导
	double      m_AnteMBAz;             //天线中心方位相对指向 弧度
	double      m_AnteMBEl; 			//天线中心俯仰相对指向弧度

	double				EnvironmentNoisePower;   //获取背景电磁环境噪声强度


};

#endif