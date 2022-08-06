/**
 * @file <src/modules/AutoMoDeCondition.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#include "AutoMoDeCondition.h"

namespace argos
{

	/****************************************/
	/****************************************/

	const std::string AutoMoDeCondition::GetDOTDescription()
	{
		std::stringstream ss;
		ss << m_strLabel;
		if (!m_mapParameters.empty())
		{
			std::map<std::string, Real>::iterator it;
			for (it = m_mapParameters.begin(); it != m_mapParameters.end(); it++)
			{
				ss << "\\n"
				   << it->first << "=" << it->second;
			}
		}
		return ss.str();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::AddParameter(const std::string &str_identifier, const Real &f_value)
	{
		m_mapParameters.insert(std::pair<std::string, Real>(str_identifier, f_value));
	}

	/****************************************/
	/****************************************/

	const UInt32 &AutoMoDeCondition::GetOrigin() const
	{
		return m_unFromBehaviourIndex;
	}

	/****************************************/
	/****************************************/

	const UInt32 &AutoMoDeCondition::GetExtremity() const
	{
		return m_unToBehaviourIndex;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetOrigin(const UInt32 &un_from)
	{
		m_unFromBehaviourIndex = un_from;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetExtremity(const UInt32 &un_to)
	{
		m_unToBehaviourIndex = un_to;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetOriginAndExtremity(const UInt32 &un_from, const UInt32 &un_to)
	{
		m_unFromBehaviourIndex = un_from;
		m_unToBehaviourIndex = un_to;
	}

	/****************************************/
	/****************************************/

	const std::string &AutoMoDeCondition::GetLabel() const
	{
		return m_strLabel;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetIndex(const UInt32 &un_index)
	{
		m_unIndex = un_index;
	}

	/****************************************/
	/****************************************/

	const UInt32 &AutoMoDeCondition::GetIndex() const
	{
		return m_unIndex;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetIdentifier(const UInt32 &un_id)
	{
		m_unIdentifier = un_id;
	}

	/****************************************/
	/****************************************/

	const UInt32 &AutoMoDeCondition::GetIdentifier() const
	{
		return m_unIdentifier;
	}

	/****************************************/
	/****************************************/

	std::map<std::string, Real> AutoMoDeCondition::GetParameters() const
	{
		return m_mapParameters;
	}

	/****************************************/
	/****************************************/

	CVector3 AutoMoDeCondition::ConvertRGBToLab(CColor c_rgb)
	{
		// lambda function to apply conditional cubic root in the XYZ to LAB algorithm
		auto f_cubic_root = [](Real f_x) -> Real
		{
			if (f_x > 0.008856)
				return pow(f_x, 1.0 / 3.0);
			else
				return 7.787 * f_x + 16.0 / 116.0;
		};
		// convert from linear RGB (not sRGB) to (CIE)XYZ
		CVector3 cRGB = CVector3(c_rgb.GetRed() / 255.0f, c_rgb.GetGreen() / 255.0f, c_rgb.GetBlue() / 255.0f);
		CVector3 cXYZ(0.0f, 0.0f, 0.0f);
		// matrix multiplication
		cXYZ.SetX(0.4124 * cRGB.GetX() + 0.3576 * cRGB.GetY() + 0.1805 * cRGB.GetZ());
		cXYZ.SetY(0.2126 * cRGB.GetX() + 0.7152 * cRGB.GetY() + 0.0722 * cRGB.GetZ());
		cXYZ.SetZ(0.0193 * cRGB.GetX() + 0.1192 * cRGB.GetY() + 0.9505 * cRGB.GetZ());
		// convert from (CIE)XYZ to (CIE)Lab
		CVector3 cLab(0.0f, 0.0f, 0.0f);
		cLab.SetX(116.0f * f_cubic_root(cXYZ.GetX() / 0.9505) - 16.0f);
		cLab.SetY(500.0f * (f_cubic_root(cXYZ.GetX() / 0.9505) - f_cubic_root(cXYZ.GetY() / 1.0)));
		cLab.SetY(200.0f * (f_cubic_root(cXYZ.GetY() / 1.0) - f_cubic_root(cXYZ.GetZ() / 1.089)));
		return cLab;
	}

	/****************************************/
	/****************************************/

	Real AutoMoDeCondition::ComputeDeltaE(CColor c_color1, CColor c_color2)
	{
		CVector3 cLab1 = ConvertRGBToLab(c_color1);
		CVector3 cLab2 = ConvertRGBToLab(c_color2);
		// compute the delta E between two colors in the LAB color space
		Real fL1 = cLab1.GetX();
		Real fA1 = cLab1.GetY();
		Real fB1 = cLab1.GetZ();
		Real fL2 = cLab2.GetX();
		Real fA2 = cLab2.GetY();
		Real fB2 = cLab2.GetZ();
		return Sqrt(pow(fL1 - fL2, 2) + pow(fA1 - fA2, 2) + pow(fB1 - fB2, 2));
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetRobotDAO(RVRDAO *pc_robot_dao)
	{
		m_pcRobotDAO = pc_robot_dao;
	}

	/****************************************/
	/****************************************/

	bool AutoMoDeCondition::EvaluateBernoulliProbability(const Real &f_probability) const
	{
		return m_pcRobotDAO->GetRandomNumberGenerator()->Bernoulli(f_probability);
	}

	/****************************************/
	/****************************************/

	// Return the color parameter
	CColor AutoMoDeCondition::GetColorParameter(const UInt32 &un_value)
	{
		CColor cColorParameter;
		switch (un_value)
		{
		case 0:
			cColorParameter = CColor::BLACK;
			break;
		case 1:
			cColorParameter = CColor::GREEN;
			break;
		case 2:
			// blue physical patches
			cColorParameter = CColor(0, 123, 194);
			break;
		case 3:
			// red physical patches
			cColorParameter = CColor(228, 53, 64);
			break;
		case 4:
			// yellow physical patches
			cColorParameter = CColor(252, 238, 33);
			break;
		case 5:
			// purple physical patches
			cColorParameter = CColor(126, 79, 154);
			break;
		case 6:
			cColorParameter = CColor::CYAN;
			break;
		default:
			cColorParameter = CColor::BLACK;
		}
		return cColorParameter;
	}

}
