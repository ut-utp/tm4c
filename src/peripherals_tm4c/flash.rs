extern crate embedded_hal;
extern crate tm4c123x;
use crate::persistent_data_management::flash::{Flash, status_error_codes};
use core::ptr::read_volatile;


// struct ARM_FLASH_INFO
// ARM_FLASH_SECTOR *	sector_info	Sector layout information (NULL=Uniform sectors)
// uint32_t	sector_count	Number of sectors.
// uint32_t	sector_size	Uniform sector size in bytes (0=sector_info used)
// uint32_t	page_size	Optimal programming page size in bytes.
// uint32_t	program_unit	Smallest programmable unit in bytes.
// uint8_t	erased_value	Contents of erased memory (usually 0xFF)
// uint8_t	reserved[3]	Reserved (must be zero)

pub const MAX_READABLE_WORDS:   usize = 128;
pub const MAX_WRITABLE_WORDS:   usize = 128;
pub const FLASH_FMA_OFFSET_MAX: u32   = 0x0003FFFF;
pub const FLASH_FMC_MERASE:      u32 =  0x00000004;  // Mass Erase Flash Memory
pub const FLASH_FMC_ERASE:      u32 =  0x00000002;  // Erase a Page of Flash Memory
pub const FLASH_FMC_WRITE:      u32 =   0x00000001;  // Write a Word into Flash Memory
pub const FLASH_BOOTCFG_KEY:    u32 =   0x00000010;  // KEY Select
pub const FLASH_FMC_WRKEY:      u32  =   0xA4420000;  // FLASH write key (KEY bit of FLASH_BOOTCFG_R set)
pub const FLASH_FMC_WRKEY2:       u32 = 0x71D50000;  // FLASH write key (KEY bit of FLASH_BOOTCFG_R cleared)

pub struct required_components{
    pub flash: tm4c123x::FLASH_CTRL,
}

pub struct TM4C_FLASH_STATUS{
	busy: u8,
	error: u8,
	reservd: u32,
}

pub struct TM4C_FLASH_SECTOR{
	start: u32,
	end: u32,
}

pub struct TM4C_FLASH_INFO{

//Data Fields
flash_sector: TM4C_FLASH_SECTOR,// *	sector_info	Sector layout information (NULL=Uniform sectors)
sector_count: u32,	//Number of sectors.
sector_size:  u32,	//Uniform sector size in bytes (0=sector_info used)
page_size:    u32,	////Optimal programming page size in bytes.
program_unit: u32,	//Smallest programmable unit in bytes.
erased_value: u32,	//Contents of erased memory (usually 0xFF)
reserved    : [u8; 3]	//Reserved (must be zero)
}



pub struct tm4c_flash_unit{
	//info: TM4C_FLASH_INFO,
  pub flash_ctrl: tm4c123x::FLASH_CTRL,
}

//All the registers we need from tm4c123x crate

// #define FLASH_FMA_R             (*((volatile uint32_t *)0x400FD000))
// #define FLASH_FMA_OFFSET_MAX    0x0003FFFF  // Address Offset max
// #define FLASH_FMD_R             (*((volatile uint32_t *)0x400FD004))
// #define FLASH_FMC_R             (*((volatile uint32_t *)0x400FD008))
// #define FLASH_FMC_WRKEY         0xA4420000  // FLASH write key (KEY bit of FLASH_BOOTCFG_R set)
// #define FLASH_FMC_WRKEY2        0x71D50000  // FLASH write key (KEY bit of FLASH_BOOTCFG_R cleared)
// #define FLASH_FMC_MERASE        0x00000004  // Mass Erase Flash Memory
// #define FLASH_FMC_ERASE         0x00000002  // Erase a Page of Flash Memory
// #define FLASH_FMC_WRITE         0x00000001  // Write a Word into Flash Memory
// #define FLASH_FMC2_R            (*((volatile uint32_t *)0x400FD020))
// #define FLASH_FMC2_WRBUF        0x00000001  // Buffered Flash Memory Write
// #define FLASH_FWBN_R            (*((volatile uint32_t *)0x400FD100))
// #define FLASH_BOOTCFG_R         (*((volatile uint32_t *)0x400FE1D0))
// #define FLASH_BOOTCFG_KEY       0x00000010  // KEY Select
impl Flash for tm4c_flash_unit{

	fn Flash_Initialize(&mut self) -> status_error_codes{

		//Nothing is really done here. THe only thing is to just make sure the power block and clocks are initialized

		status_error_codes::ARM_DRIVER_ERROR_SPECIFIC

	}
	fn Flash_Uninitialize(&mut self) -> status_error_codes{

		status_error_codes::ARM_DRIVER_ERROR_SPECIFIC

	}
	fn Flash_ReadData(&self, addr: u32, num_items: u8) -> status_error_codes{
    let addr_ptr: *const u32 = addr as (*const u32);
    let mut result=0;
    unsafe{result = read_volatile(addr_ptr);}
		status_error_codes::ARM_DRIVER_OK_READ(result)

	}


    fn Flash_ReadSector(&self, addr: u32) -> Option<[u32; MAX_READABLE_WORDS]>{
    let addr_ptr: *const u32 = addr as (*const u32);

    if(addr%512==0){
    let mut result: [u32; MAX_READABLE_WORDS]=[0; MAX_READABLE_WORDS];
    for i in 0..MAX_READABLE_WORDS {
      unsafe{let resultd = self.Flash_ReadData(addr+4*i as u32, 1);
              match resultd{
                status_error_codes::ARM_DRIVER_OK_READ(out) =>{
                  result[i] = out;
                },
                _=>{

                },

              }

      }
    }
    
    Some(result)
  }
  else{
    None
  }
  }

  fn Flash_WriteWord(&mut self, addr: u32, data: u32)-> status_error_codes{

    let mut flashkey: u32 =0;
   if(WriteAddrValid(addr)){
    let sec_data = self.Flash_ReadSector(addr - (addr%512));
    let mut sec_data_res: [u32; 128] = [0; 128];
    match  sec_data {
      Some(expr) => {
        sec_data_res = expr;
      },
      None => {},
    }

    sec_data_res[((addr%512u32)/4u32) as usize] = data;

    //let p = unsafe { &*tm4c123x::FLASH_CTRL::ptr() };
     while((self.flash_ctrl.fmc.read().bits()&(FLASH_FMC_WRITE|FLASH_FMC_ERASE|FLASH_FMC_MERASE)) > 0){
  //                //  TODO: return ERROR if this takes too long
     };


     self.Flash_EraseSector(addr - (addr%512));

     for i in 0..128 {
     self.flash_ctrl.fmd.write(|w| unsafe{w.bits(sec_data_res[i])});
     self.flash_ctrl.fma.write(|w| unsafe{w.bits(addr - (addr%512)+4*i as u32)});
     if((self.flash_ctrl.bootcfg.read().bits()&FLASH_BOOTCFG_KEY)>0){          // by default, the key is 0xA442
       flashkey = FLASH_FMC_WRKEY;
     } else{                                         // otherwise, the key is 0x71D5
       flashkey = FLASH_FMC_WRKEY2;
     }

     self.flash_ctrl.fmc.write(|w| unsafe{w.bits(flashkey|FLASH_FMC_WRITE)});
     //FLASH_FMC_R = (flashkey|FLASH_FMC_WRITE);       // start writing
     while((self.flash_ctrl.fmc.read().bits()&FLASH_FMC_WRITE) > 0){
  //                TODO: return ERROR if this takes too long
     };           // wait for completion (~3 to 4 usec)
     }


     
     status_error_codes::ARM_DRIVER_OK_WRITE
  }

   else{

    status_error_codes::ARM_DRIVER_ERROR_SPECIFIC
  }
}


	fn Flash_ProgramData(&mut self, addr: u32, data: [usize; MAX_WRITABLE_WORDS])-> status_error_codes{

  // uint16_t successfulWrites = 0;
  // while((successfulWrites < count) && (Flash_Write(addr + 4*successfulWrites, source[successfulWrites]) == NOERROR)){
  //   successfulWrites = successfulWrites + 1;
  // }
  // return successfulWrites;
  let mut successfulWrites: u32 =0;
   while((successfulWrites < (MAX_WRITABLE_WORDS as u32) )){
    // let current_addr = addr + (4*successfulWrites);
     self.Flash_WriteWord(addr + (4*successfulWrites), data[successfulWrites as usize] as u32);
     successfulWrites = successfulWrites + 1;
   } 

   status_error_codes::ARM_DRIVER_OK_WRITE
}
	fn Flash_EraseSector(&mut self, addr: u32) -> status_error_codes{

    let mut flashkey: u32 =0;
   if(EraseAddrValid(addr)){
   // let p = unsafe { &*tm4c123x::FLASH_CTRL::ptr() };
     while((self.flash_ctrl.fmc.read().bits()&(FLASH_FMC_WRITE|FLASH_FMC_ERASE|FLASH_FMC_MERASE)) > 0){
  //                //  TODO: return ERROR if this takes too long
     };
     self.flash_ctrl.fma.write(|w| unsafe{w.bits(addr)});
     if((self.flash_ctrl.bootcfg.read().bits()&FLASH_BOOTCFG_KEY)>0){          // by default, the key is 0xA442
       flashkey = FLASH_FMC_WRKEY;
     } else{                                         // otherwise, the key is 0x71D5
       flashkey = FLASH_FMC_WRKEY2;
     }

     self.flash_ctrl.fmc.write(|w| unsafe{w.bits(flashkey|FLASH_FMC_ERASE)});
     while((self.flash_ctrl.fmc.read().bits()&FLASH_FMC_ERASE) > 0){
  //                TODO: return ERROR if this takes too long
     };           // wait for completion (~3 to 4 usec)
     status_error_codes::ARM_DRIVER_OK_ERASE

     

  }
  else{

		status_error_codes::ARM_DRIVER_ERROR_SPECIFIC
  }

	}
	fn Flash_EraseChip(&mut self) -> status_error_codes{

		//call erase sector on all sectors

		status_error_codes::ARM_DRIVER_ERROR_SPECIFIC

	}
	fn Flash_GetStatus(&mut self){

	}
	fn Flash_GetInfo(&mut self){

	}

}

fn WriteAddrValid(addr: u32) -> bool{
  // check if address offset works for writing
  // must be 4-byte aligned
  (((addr % 4) == 0) && (addr <= FLASH_FMA_OFFSET_MAX))
}

fn EraseAddrValid(addr: u32) -> bool{
  // check if address offset works for writing
  // must be 4-byte aligned
  (((addr % 1024) == 0) && (addr <= FLASH_FMA_OFFSET_MAX))
}