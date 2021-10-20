//
// Created by Paul on 19-Oct-21.
//

#include <string.h>
#include "component_storage.h"

QSPI_HandleTypeDef hqspi;
DMA_HandleTypeDef hdma_quadspi;

typedef struct
{
	struct
	{
		QSPI_CommandTypeDef Reset;              // 0x9F   1C + 1D + 3BYTES
		QSPI_CommandTypeDef ReadID;             // 0x9F   1C + 1D + 3BYTES
		QSPI_CommandTypeDef SetFeature;         // 0x1F   1C + 1A + 1D
		QSPI_CommandTypeDef GetFeature;         // 0x0F   1C + 1A + 1D
		QSPI_CommandTypeDef WriteEnable;        // 0x06   1C
		QSPI_CommandTypeDef ProgramLoad;        // 0x32   1 + 2 BYTES (0) + 4224
		QSPI_CommandTypeDef ProgramExecute;     // 0x10   1 + 3 BYTES (BLOCK + PAGE)
		QSPI_CommandTypeDef BlockErase;         // 0x10   1 + 3 BYTES (BLOCK + PAGE)
		QSPI_CommandTypeDef ReadCellArray;      // 0x13   1 + 3 BYTES (BLOCK + PAGE)
		QSPI_CommandTypeDef ReadBuffer;         // 0x6B   1 + 3 BYTES (0)

	} Commands;
} CONTEXT_typedef;
static CONTEXT_typedef this;


static void InitializePeripheral();

static void InitializeCommands();
void printfArray(uint8_t* data);

uint32_t mutex = 1;

void COMPONENT_STORAGE_Initialize()
{
	InitializePeripheral();
	InitializeCommands();


	uint8_t commandBuffer[4224 * 2];
	uint8_t readBuffer[4224 * 2];
	HAL_QSPI_Command(&hqspi, &this.Commands.ReadID, 1000);

	HAL_QSPI_Receive(&hqspi, commandBuffer, 1000);
	for (int i = 0; i < 3; ++i)
	{
		printf("%02X ", commandBuffer[i]);
	}
	printf("\n");
	memset(commandBuffer, 0, 128);
//  #################  RESET DEVICE
	HAL_QSPI_Command(&hqspi, &this.Commands.Reset, 1000);
	HAL_Delay(1000);

//  #################  UNLOCK DEVICE
	this.Commands.GetFeature.AlternateBytes = 0xA0;
	commandBuffer[0] = 0xFA;
	HAL_QSPI_Command(&hqspi, &this.Commands.GetFeature, 1000);
	HAL_QSPI_Receive(&hqspi, commandBuffer, 1000);
	printf("LOCKED BITS : %02X\n", commandBuffer[0]);

	this.Commands.SetFeature.AlternateBytes = 0xA0;
	commandBuffer[0] = 0x00;
	HAL_QSPI_Command(&hqspi, &this.Commands.SetFeature, 1000);
	HAL_QSPI_Transmit(&hqspi, commandBuffer, 1000);

	this.Commands.GetFeature.AlternateBytes = 0xA0;
	commandBuffer[0] = 0xFA;
	HAL_QSPI_Command(&hqspi, &this.Commands.GetFeature, 1000);
	HAL_QSPI_Receive(&hqspi, commandBuffer, 1000);
	printf("LOCK BITS : %02X\n", commandBuffer[0]);

//  #################  UNLOCK DEVICE
	this.Commands.GetFeature.AlternateBytes = 0xB0;
	commandBuffer[0] = 0xFA;
	HAL_QSPI_Command(&hqspi, &this.Commands.GetFeature, 1000);
	HAL_QSPI_Receive(&hqspi, commandBuffer, 1000);
	printf("HOLD_D BIT : %02X\n", commandBuffer[0]);

	this.Commands.SetFeature.AlternateBytes = 0xB0;
	commandBuffer[0] = 0x13;
	HAL_QSPI_Command(&hqspi, &this.Commands.SetFeature, 1000);
	HAL_QSPI_Transmit(&hqspi, commandBuffer, 1000);

	this.Commands.GetFeature.AlternateBytes = 0xB0;
	commandBuffer[0] = 0xFA;
	HAL_QSPI_Command(&hqspi, &this.Commands.GetFeature, 1000);
	HAL_QSPI_Receive(&hqspi, commandBuffer, 1000);
	printf("HOLD_D BIT : %02X\n", commandBuffer[0]);

	this.Commands.GetFeature.AlternateBytes = 0xC0;
	commandBuffer[0] = 0xFA;
	HAL_QSPI_Command(&hqspi, &this.Commands.GetFeature, 1000);
	HAL_QSPI_Receive(&hqspi, commandBuffer, 1000);
	printf("INITIAL WEL BIT : %02X\n", commandBuffer[0]);

	HAL_QSPI_Command(&hqspi, &this.Commands.WriteEnable, 1000);

	commandBuffer[0] = 0xFA;
	this.Commands.GetFeature.AlternateBytes = 0xC0;
	HAL_QSPI_Command(&hqspi, &this.Commands.GetFeature, 1000);
	HAL_QSPI_Receive(&hqspi, commandBuffer, 1000);
	printf("FINAL WEL BIT : %02X\n", commandBuffer[0]);


	for (int i = 0; i < 4224; ++i)
	{
		commandBuffer[i] = i % 256;
		if(commandBuffer[i] == 0x0F || commandBuffer[i] == 0xF1) commandBuffer[i] = 0xAA;
		readBuffer[i] = 0x00;
	}
	uint8_t result;

//	this.Commands.BlockErase.Address = 1;
//	HAL_QSPI_Command(&hqspi, &this.Commands.WriteEnable, 1000);
//	HAL_QSPI_Command(&hqspi, &this.Commands.BlockErase, HAL_MAX_DELAY);
//	while(1)
//	{
//		commandBuffer[0] = 0xFF;
//		this.Commands.GetFeature.AlternateBytes = 0xC0;
//		HAL_QSPI_Command(&hqspi, &this.Commands.GetFeature, 1000);
//		HAL_QSPI_Receive(&hqspi, commandBuffer, 1000);
//		printf("OP STATUS : %02X\n", commandBuffer[0]);
//		if(commandBuffer[0] == 0x00)
//		{
//			break;
//		}
//	}

	HAL_QSPI_Command(&hqspi, &this.Commands.WriteEnable, 1000);

//  #################  LOAD BUFFER

	printf("LOADING DATA\n");
	result = HAL_QSPI_Command(&hqspi, &this.Commands.ProgramLoad, HAL_MAX_DELAY);
	printf("SENT COMMAND %02X\n", result);
	result = HAL_QSPI_Transmit(&hqspi, commandBuffer, HAL_MAX_DELAY);
	printf("DATA LOADED %02X\n", result);


//  #################  EXECUTE BUFFER
	this.Commands.ProgramExecute.Address = 0;
	result = HAL_QSPI_Command(&hqspi, &this.Commands.ProgramExecute, HAL_MAX_DELAY);
	printf("EXECUTE COMMAND %02X\n", result);

	while(1)
	{
		HAL_QSPI_Command(&hqspi, &this.Commands.GetFeature, HAL_MAX_DELAY);
		result = HAL_QSPI_Receive(&hqspi, commandBuffer, 1000);
		if((commandBuffer[0] & (1 << 1)) == 0)
		{
			printf("OP STATUS : %02X - %02X\n", commandBuffer[0], result);
			break;
		}
		else if(commandBuffer[0] == 0x20)
		{
			printf("ERROR ECC\n");
			break;
		}
	}


		//	  #################  READ CELL ARRAY
		this.Commands.ReadCellArray.Address = 0;
		result = HAL_QSPI_Command_IT(&hqspi, &this.Commands.ReadCellArray);
		while(mutex);

		printf("CELL READ %02X\n", result);
	HAL_Delay(1000);
		this.Commands.GetFeature.AlternateBytes = 0xC0;
		HAL_QSPI_Command(&hqspi, &this.Commands.GetFeature, HAL_MAX_DELAY);
		while(1)
		{
			result = HAL_QSPI_Receive(&hqspi, commandBuffer, 1000);
			if((commandBuffer[0] & (1 << 1)) == 0)
			{
				printf("OP STATUS : %02X - %02X\n", commandBuffer[0], result);
				break;
			}
			else if(commandBuffer[0] == 0x20)
			{
				printf("ERROR ECC\n");
				break;
			}
		}

		mutex = 1;
		HAL_QSPI_Command(&hqspi, &this.Commands.ReadBuffer, HAL_MAX_DELAY);
		result = HAL_QSPI_Receive_DMA(&hqspi, readBuffer);
		while(mutex);
		printfArray(readBuffer);


}

void printfArray(uint8_t* data)
{
	uint8_t counter = 0;
	for (int i = 0; i < 4224; ++i)
	{
		printf("%02X ", *data++);
		if(++counter == 48)
		{
			printf("\n");
			counter = 0;
		}

	}
}

void                  HAL_QSPI_CmdCpltCallback      (QSPI_HandleTypeDef *hqspi)
{
	mutex = 0;
}

void                  HAL_QSPI_TxCpltCallback       (QSPI_HandleTypeDef *hqspi)
{
	mutex = 0;
}

void                  HAL_QSPI_RxCpltCallback       (QSPI_HandleTypeDef *hqspi)
{
	mutex=  0;
}

void InitializePeripheral()
{
	hqspi.Instance = QUADSPI;
	hqspi.Init.ClockPrescaler = 2;
	hqspi.Init.FifoThreshold = 4;
	hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
	hqspi.Init.FlashSize = 29;
	hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_8_CYCLE;
	hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
	if (HAL_QSPI_Init(&hqspi) != HAL_OK)
	{
		Error_Handler();
	}
}


void InitializeCommands()
{
	this.Commands.ReadID.Instruction = 0x9F;                             /* Specifies the Instruction to be sent This parameter can be a value (8-bit) between 0x00 and 0xFF */
	this.Commands.ReadID.InstructionMode = QSPI_INSTRUCTION_1_LINE;          /* Specifies the Instruction Mode This parameter can be a value of @ref QSPI_InstructionMode */
	this.Commands.ReadID.Address = 0;                                /* Specifies the Address to be sent (Size from 1 to 4 bytes according AddressSize) This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */
	this.Commands.ReadID.AddressMode = QSPI_ADDRESS_NONE;                /* Specifies the Address Mode This parameter can be a value of @ref QSPI_AddressMode */
	this.Commands.ReadID.AddressSize = QSPI_ADDRESS_16_BITS;             /* Specifies the Address Size This parameter can be a value of @ref QSPI_AddressSize */
	this.Commands.ReadID.AlternateBytes = 1;                                /* Specifies the Alternate Bytes to be sent (Size from 1 to 4 bytes according AlternateBytesSize) This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */
	this.Commands.ReadID.AlternateByteMode = QSPI_ALTERNATE_BYTES_1_LINE;      /* Specifies the Alternate Bytes Mode This parameter can be a value of @ref QSPI_AlternateBytesMode */
	this.Commands.ReadID.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;      /* Specifies the Alternate Bytes Size This parameter can be a value of @ref QSPI_AlternateBytesSize */
	this.Commands.ReadID.DummyCycles = 0;                                /* Specifies the Number of Dummy Cycles. This parameter can be a number between 0 and 31 */
	this.Commands.ReadID.DataMode = QSPI_DATA_1_LINE;                 /* Specifies the Data Mode (used for dummy cycles and data phases) This parameter can be a value of @ref QSPI_DataMode */
	this.Commands.ReadID.NbData = 3;                                /* Specifies the number of data to transfer. (This is the number of bytes) This parameter can be any value between 0 and 0xFFFFFFFF (0 means undefined length until end of memory)*/
	this.Commands.ReadID.DdrMode = QSPI_DDR_MODE_DISABLE;            /* Specifies the double data rate mode for address, alternate byte and data phase This parameter can be a value of @ref QSPI_DdrMode */
	this.Commands.ReadID.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;    /* Specifies the send instruction only once mode This parameter can be a value of @ref QSPI_SIOOMode */

	this.Commands.Reset.Instruction = 0xFF;
	this.Commands.Reset.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	this.Commands.Reset.Address = 0;
	this.Commands.Reset.AddressMode = QSPI_ADDRESS_NONE;
	this.Commands.Reset.AddressSize = QSPI_ADDRESS_16_BITS;
	this.Commands.Reset.AlternateBytes = 0x00;
	this.Commands.Reset.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	this.Commands.Reset.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
	this.Commands.Reset.DummyCycles = 0;
	this.Commands.Reset.DataMode = QSPI_DATA_NONE;
	this.Commands.Reset.NbData = 0;
	this.Commands.Reset.DdrMode = QSPI_DDR_MODE_DISABLE;
	this.Commands.Reset.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	this.Commands.SetFeature.Instruction = 0x1F;
	this.Commands.SetFeature.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	this.Commands.SetFeature.Address = 0;
	this.Commands.SetFeature.AddressMode = QSPI_ADDRESS_NONE;
	this.Commands.SetFeature.AddressSize = QSPI_ADDRESS_16_BITS;
	this.Commands.SetFeature.AlternateBytes = 0xB0;
	this.Commands.SetFeature.AlternateByteMode = QSPI_ALTERNATE_BYTES_1_LINE;
	this.Commands.SetFeature.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
	this.Commands.SetFeature.DummyCycles = 0;
	this.Commands.SetFeature.DataMode = QSPI_DATA_1_LINE;
	this.Commands.SetFeature.NbData = 1;
	this.Commands.SetFeature.DdrMode = QSPI_DDR_MODE_DISABLE;
	this.Commands.SetFeature.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	this.Commands.GetFeature.Instruction = 0x0F;
	this.Commands.GetFeature.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	this.Commands.GetFeature.Address = 0;
	this.Commands.GetFeature.AddressMode = QSPI_ADDRESS_NONE;
	this.Commands.GetFeature.AddressSize = QSPI_ADDRESS_16_BITS;
	this.Commands.GetFeature.AlternateBytes = 0xB0;
	this.Commands.GetFeature.AlternateByteMode = QSPI_ALTERNATE_BYTES_1_LINE;
	this.Commands.GetFeature.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
	this.Commands.GetFeature.DummyCycles = 0;
	this.Commands.GetFeature.DataMode = QSPI_DATA_1_LINE;
	this.Commands.GetFeature.NbData = 1;
	this.Commands.GetFeature.DdrMode = QSPI_DDR_MODE_DISABLE;
	this.Commands.GetFeature.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	this.Commands.WriteEnable.Instruction = 0x06;
	this.Commands.WriteEnable.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	this.Commands.WriteEnable.Address = 0;
	this.Commands.WriteEnable.AddressMode = QSPI_ADDRESS_NONE;
	this.Commands.WriteEnable.AddressSize = QSPI_ADDRESS_16_BITS;
	this.Commands.WriteEnable.AlternateBytes = 0;
	this.Commands.WriteEnable.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	this.Commands.WriteEnable.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
	this.Commands.WriteEnable.DummyCycles = 0;
	this.Commands.WriteEnable.DataMode = QSPI_DATA_NONE;
	this.Commands.WriteEnable.NbData = 0;
	this.Commands.WriteEnable.DdrMode = QSPI_DDR_MODE_DISABLE;
	this.Commands.WriteEnable.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	this.Commands.ProgramLoad.Instruction = 0x32;
	this.Commands.ProgramLoad.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	this.Commands.ProgramLoad.Address = 0;
	this.Commands.ProgramLoad.AddressMode = QSPI_ADDRESS_1_LINE;
	this.Commands.ProgramLoad.AddressSize = QSPI_ADDRESS_16_BITS;
	this.Commands.ProgramLoad.AlternateBytes = (uint16_t)0;
	this.Commands.ProgramLoad.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	this.Commands.ProgramLoad.AlternateBytesSize = QSPI_ALTERNATE_BYTES_16_BITS;
	this.Commands.ProgramLoad.DummyCycles = 0;
	this.Commands.ProgramLoad.DataMode = QSPI_DATA_4_LINES;
	this.Commands.ProgramLoad.NbData = 4224;
	this.Commands.ProgramLoad.DdrMode = QSPI_DDR_MODE_DISABLE;
	this.Commands.ProgramLoad.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	this.Commands.ReadBuffer.Instruction = 0x6B;
	this.Commands.ReadBuffer.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	this.Commands.ReadBuffer.Address = 0;
	this.Commands.ReadBuffer.AddressMode = QSPI_ADDRESS_1_LINE;
	this.Commands.ReadBuffer.AddressSize = QSPI_ADDRESS_16_BITS;
	this.Commands.ReadBuffer.AlternateBytes = (uint16_t)0;
	this.Commands.ReadBuffer.AlternateByteMode = QSPI_ALTERNATE_BYTES_1_LINE;
	this.Commands.ReadBuffer.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
	this.Commands.ReadBuffer.DummyCycles = 0;
	this.Commands.ReadBuffer.DataMode = QSPI_DATA_4_LINES;
	this.Commands.ReadBuffer.NbData = 4224;
	this.Commands.ReadBuffer.DdrMode = QSPI_DDR_MODE_DISABLE;
	this.Commands.ReadBuffer.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	this.Commands.ProgramExecute.Instruction = 0x10;
	this.Commands.ProgramExecute.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	this.Commands.ProgramExecute.Address = 0;
	this.Commands.ProgramExecute.AddressMode = QSPI_ADDRESS_1_LINE;
	this.Commands.ProgramExecute.AddressSize = QSPI_ADDRESS_24_BITS;
	this.Commands.ProgramExecute.AlternateBytes = 0;
	this.Commands.ProgramExecute.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	this.Commands.ProgramExecute.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
	this.Commands.ProgramExecute.DummyCycles = 0;
	this.Commands.ProgramExecute.DataMode = QSPI_DATA_NONE;
	this.Commands.ProgramExecute.NbData = 0;
	this.Commands.ProgramExecute.DdrMode = QSPI_DDR_MODE_DISABLE;
	this.Commands.ProgramExecute.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	this.Commands.ReadCellArray.Instruction = 0x13;
	this.Commands.ReadCellArray.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	this.Commands.ReadCellArray.Address = 0;
	this.Commands.ReadCellArray.AddressMode = QSPI_ADDRESS_1_LINE;
	this.Commands.ReadCellArray.AddressSize = QSPI_ADDRESS_24_BITS;
	this.Commands.ReadCellArray.AlternateBytes = 0;
	this.Commands.ReadCellArray.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	this.Commands.ReadCellArray.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
	this.Commands.ReadCellArray.DummyCycles = 0;
	this.Commands.ReadCellArray.DataMode = QSPI_DATA_NONE;
	this.Commands.ReadCellArray.NbData = 0;
	this.Commands.ReadCellArray.DdrMode = QSPI_DDR_MODE_DISABLE;
	this.Commands.ReadCellArray.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	this.Commands.BlockErase.Instruction = 0xD8;
	this.Commands.BlockErase.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	this.Commands.BlockErase.Address = 0;
	this.Commands.BlockErase.AddressMode = QSPI_ADDRESS_1_LINE;
	this.Commands.BlockErase.AddressSize = QSPI_ADDRESS_24_BITS;
	this.Commands.BlockErase.AlternateBytes = 0;
	this.Commands.BlockErase.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	this.Commands.BlockErase.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
	this.Commands.BlockErase.DummyCycles = 0;
	this.Commands.BlockErase.DataMode = QSPI_DATA_NONE;
	this.Commands.BlockErase.NbData = 0;
	this.Commands.BlockErase.DdrMode = QSPI_DDR_MODE_DISABLE;
	this.Commands.BlockErase.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;


}