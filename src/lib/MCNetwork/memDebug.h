#pragma once

#ifdef MC_MEM_DEBUG
#define MC_ON_ENTRY(FUNC) long mem_on_entry=ESP.getFreeHeap();
#define MC_ON_EXIT(FUNC)  long mem_on_exit=ESP.getFreeHeap(); if (mem_on_entry != mem_on_exit) log4MC::vlogf(LOG_DEBUG, "Diff in memory detected in function %s, %d",FUNC,(mem_on_exit - mem_on_entry));
#else
#define MC_ON_ENTRY(FUNC)
#define MC_ON_EXIT(FUNC)
#endif