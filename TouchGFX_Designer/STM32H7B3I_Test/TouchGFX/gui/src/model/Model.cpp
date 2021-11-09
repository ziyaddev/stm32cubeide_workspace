#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include "stm32h7xx_hal.h"

extern __IO uint16_t uhADCxConvertedData;

Model::Model() : modelListener(0)
{

}

void Model::tick()
{
	modelListener->UpdateGraph(uhADCxConvertedData * 3300 / 65535);
}
