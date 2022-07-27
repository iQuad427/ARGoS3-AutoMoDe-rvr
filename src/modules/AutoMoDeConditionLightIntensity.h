/**
 * @file <src/modules/AutoMoDeConditionLightIntensity.h>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#ifndef AUTOMODE_CONDITION_LIGHT_INTENSITY_H
#define AUTOMODE_CONDITION_LIGHT_INTENSITY_H

#include "AutoMoDeCondition.h"

namespace argos
{
	class AutoMoDeConditionLightIntensity : public AutoMoDeCondition
	{
	public:
		AutoMoDeConditionLightIntensity();
		virtual ~AutoMoDeConditionLightIntensity();

		AutoMoDeConditionLightIntensity(AutoMoDeConditionLightIntensity *pc_condition);
		virtual AutoMoDeConditionLightIntensity *Clone();

		virtual bool Verify();
		virtual void Reset();
		virtual void Init();

	private:
		Real m_fParameterEta;
		UInt8 m_unParameterXi;
	};
}

#endif
