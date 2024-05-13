#!/usr/bin/env python
## Copyright (c) 2021, Alliance for Open Media. All rights reserved
##
## This source code is subject to the terms of the BSD 3-Clause Clear License and the
## Alliance for Open Media Patent License 1.0. If the BSD 3-Clause Clear License was
## not distributed with this source code in the LICENSE file, you can obtain it
## at aomedia.org/license/software-license/bsd-3-c-c/.  If the Alliance for Open Media Patent
## License 1.0 was not distributed with this source code in the PATENTS file, you
## can obtain it at aomedia.org/license/patent-license/.
##
__author__ = "maggie.sun@intel.com, ryanlei@meta.com"

import os
import re
import sys
import argparse
import subprocess
from CalculateQualityMetrics import CalculateQualityMetric, GatherQualityMetrics
from Utils import GetShortContentName, CreateNewSubfolder, SetupLogging, \
     Cleanfolder, CreateClipList, GetEncLogFile, GatherPerfInfo, \
     GetRDResultCsvFile, GatherPerframeStat, GatherInstrCycleInfo, DeleteFile, md5
import Utils
from Config import LogLevels, FrameNum, TEST_CONFIGURATIONS, QPs, WorkPath, \
     Path_RDResults, LoggerName, QualityList, MIN_GOP_LENGTH, UsePerfUtil, \
     EnableTimingInfo, CodecNames, EnableMD5, HEVC_QPs, SUFFIX, EnableParallelGopEncoding, \
     GOP_SIZE
from EncDecUpscale import Encode, Decode, GetBitstreamFile

###############################################################################
##### Helper Functions ########################################################
def CleanIntermediateFiles():
    folders = [Path_DecodedYuv, Path_CfgFiles]
    for folder in folders:
        Cleanfolder(folder)

def GetBsReconFileName(EncodeMethod, CodecName, EncodePreset, test_cfg, clip, QP):
    basename = GetShortContentName(clip.file_name, False)
    suffix = SUFFIX[CodecName]
    filename = "%s_%s_%s_%s_Preset_%s_QP_%d%s" % \
               (basename, EncodeMethod, CodecName, test_cfg, EncodePreset, QP, suffix)
    bs = os.path.join(Path_Bitstreams, filename)
    filename = "%s_%s_%s_%s_Preset_%s_QP_%d_Decoded.y4m" % \
               (basename, EncodeMethod, CodecName, test_cfg, EncodePreset, QP)
    dec = os.path.join(Path_DecodedYuv, filename)
    return bs, dec

def setupWorkFolderStructure():
    global Path_Bitstreams, Path_DecodedYuv, Path_QualityLog, Path_TestLog,\
           Path_CfgFiles, Path_TimingLog, Path_EncLog, Path_CmdLog
    Path_Bitstreams = CreateNewSubfolder(WorkPath, "bitstreams")
    Path_DecodedYuv = CreateNewSubfolder(WorkPath, "decodedYUVs")
    Path_QualityLog = CreateNewSubfolder(WorkPath, "qualityLogs")
    Path_TestLog = CreateNewSubfolder(WorkPath, "testLogs")
    Path_CfgFiles = CreateNewSubfolder(WorkPath, "configFiles")
    Path_TimingLog = CreateNewSubfolder(WorkPath, "perfLogs")
    Path_EncLog = CreateNewSubfolder(WorkPath, "encLogs")
    Path_CmdLog = CreateNewSubfolder(WorkPath, "cmdLogs")

###############################################################################
######### Major Functions #####################################################
def CleanUp_workfolders():
    folders = [Path_Bitstreams, Path_DecodedYuv, Path_QualityLog,
               Path_TestLog, Path_CfgFiles, Path_TimingLog, Path_EncLog]
    for folder in folders:
        Cleanfolder(folder)

def Run_Encode_Test(test_cfg, clip, codec, method, preset, LogCmdOnly = False):
    Utils.Logger.info("start running %s encode tests with %s"
                      % (test_cfg, clip.file_name))
    QPSet = QPs[test_cfg]
    if codec == "hevc":
        QPSet = HEVC_QPs[test_cfg]

    for QP in QPSet:
        Utils.Logger.info("start encode with QP %d" % (QP))
        #encode
        total_frame = FrameNum[test_cfg]
        JobName = '%s_%s_%s_%s_Preset_%s_QP_%d' % \
                    (GetShortContentName(clip.file_name, False),
                    method, codec, test_cfg, preset, QP)
        if LogCmdOnly:
            Utils.CmdLogger.write("============== %s Job Start =================\n"%JobName)
        bsFile = Encode(method, codec, preset, clip, test_cfg, QP,
                        total_frame, Path_Bitstreams, Path_TimingLog,
                        Path_EncLog, 0, LogCmdOnly)
        Utils.Logger.info("start decode file %s" % os.path.basename(bsFile))
        #decode
        decodedYUV = Decode(clip, method, test_cfg, codec, bsFile, Path_DecodedYuv, Path_TimingLog,
                            False, LogCmdOnly)
        #calcualte quality distortion
        Utils.Logger.info("start quality metric calculation")
        CalculateQualityMetric(clip.file_path, FrameNum[test_cfg], decodedYUV,
                                clip.fmt, clip.width, clip.height, clip.bit_depth,
                                Path_QualityLog, LogCmdOnly)
        if SaveMemory:
            DeleteFile(decodedYUV, LogCmdOnly)
        Utils.Logger.info("finish running encode with QP %d" % (QP))
        if LogCmdOnly:
            Utils.CmdLogger.write("============== %s Job End ===================\n\n"%JobName)

def Run_Parallel_Encode_Test(test_cfg, clip, codec, method, preset, LogCmdOnly = False):
    if EnableParallelGopEncoding:
        Utils.Logger.info("start running %s encode tests with %s"
                          % (test_cfg, clip.file_name))
        QPSet = QPs[test_cfg]
        if codec == "hevc":
            QPSet = HEVC_QPs[test_cfg]

        for QP in QPSet:
            Utils.Logger.info("start encode with QP %d" % (QP))
            total_frame = FrameNum[test_cfg]    
            # Encode
            start_frame = 0
            while start_frame < total_frame:
                #encode
                num_frames = GOP_SIZE
                if (start_frame + num_frames) > total_frame:
                    num_frames = total_frame - start_frame

                JobName = '%s_%s_%s_%s_Preset_%s_QP_%d_start_%d_frames_%d' % \
                          (GetShortContentName(clip.file_name, False),
                          method, codec, test_cfg, preset, QP, start_frame, num_frames)
                if LogCmdOnly:
                    Utils.CmdLogger.write("============== %s Job Start =================\n"%JobName)
                
                bsFile = Encode(method, codec, preset, clip, test_cfg, QP,
                                num_frames, Path_Bitstreams, Path_TimingLog,
                                Path_EncLog, start_frame, LogCmdOnly)
                start_frame += num_frames

                if LogCmdOnly:
                    Utils.CmdLogger.write("============== %s Job End ===================\n\n"%JobName)
    else:
        Utils.Logger.error("parallel encode function can only be used with Parallel Gop Encoding mode")

def Run_Decode_Test(test_cfg, clip, codec, method, preset, LogCmdOnly = False):
    if EnableParallelGopEncoding:
        Utils.Logger.info("start running %s decode tests with %s"
                          % (test_cfg, clip.file_name))
        QPSet = QPs[test_cfg]
        if codec == "hevc":
            QPSet = HEVC_QPs[test_cfg]

        for QP in QPSet:
            Utils.Logger.info("start decode  with QP %d" % (QP))
            total_frame = FrameNum[test_cfg]   
            # decode
            JobName = '%s_%s_%s_%s_Preset_%s_QP_%d' % \
                          (GetShortContentName(clip.file_name, False),
                          method, codec, test_cfg, preset, QP)
            if LogCmdOnly:
                Utils.CmdLogger.write("============== %s Job Start =================\n"%JobName)
            
            bsfile = '%s/%s_%s_%s_%s_Preset_%s_QP_%d.obu' % \
                    (Path_Bitstreams, GetShortContentName(clip.file_name, False),
                    method, codec, test_cfg, preset, QP)

            decodedYUV = Decode(clip, method, test_cfg, codec, bsfile, Path_DecodedYuv, Path_TimingLog,
                                False, LogCmdOnly)
            #calcualte quality distortion
            Utils.Logger.info("start quality metric calculation")
            CalculateQualityMetric(clip.file_path, FrameNum[test_cfg], decodedYUV,
                                   clip.fmt, clip.width, clip.height, clip.bit_depth,
                                   Path_QualityLog, LogCmdOnly)
            if SaveMemory:
                DeleteFile(decodedYUV, LogCmdOnly)
            Utils.Logger.info("finish running encode with QP %d" % (QP))
            if LogCmdOnly:
                Utils.CmdLogger.write("============== %s Job End ===================\n\n"%JobName)
    else:
        Utils.Logger.error("decode function can only be used with Parallel Gop Encoding mode")

def Run_Concatenate_Test(test_cfg, clip, codec, method, preset, LogCmdOnly = False):
    if EnableParallelGopEncoding:
        Utils.Logger.info("start running %s concatenate tests with %s"
                      % (test_cfg, clip.file_name))
        QPSet = QPs[test_cfg]
        if codec == "hevc":
            QPSet = HEVC_QPs[test_cfg]

        for QP in QPSet:
            Utils.Logger.info("start encode with QP %d" % (QP))
            total_frame = FrameNum[test_cfg]; start_frame = 0
            JobName = '%s_%s_%s_%s_Preset_%s_QP_%d' % \
                          (GetShortContentName(clip.file_name, False),
                          method, codec, test_cfg, preset, QP)
            cmd = "cat "    
            while start_frame < total_frame:
                num_frames = GOP_SIZE
                if (start_frame + num_frames) > total_frame:
                    num_frames = total_frame - start_frame

                bsfile = GetBitstreamFile(method, codec, test_cfg, preset, clip.file_path,
                              QP, start_frame, num_frames, Path_Bitstreams)
                cmd += " %s " % bsfile
                start_frame += num_frames
            
            bsfile = '%s_%s_%s_%s_Preset_%s_QP_%d.obu' % \
                          (GetShortContentName(clip.file_name, False),
                          method, codec, test_cfg, preset, QP)
                
            cmd += " > %s/%s" % (Path_Bitstreams, bsfile)

            if LogCmdOnly:
                Utils.CmdLogger.write("%s\n" % cmd)
            else:
                subprocess.call(cmd, shell=True)

def GenerateSummaryRDDataFile(EncodeMethod, CodecName, EncodePreset,
                              test_cfg, clip_list, log_path, missing):
    Utils.Logger.info("start saving RD results to excel file.......")
    if not os.path.exists(Path_RDResults):
        os.makedirs(Path_RDResults)

    csv_file, perframe_csvfile = GetRDResultCsvFile(EncodeMethod, CodecName, EncodePreset, test_cfg)
    csv = open(csv_file, 'wt')
    # "TestCfg,EncodeMethod,CodecName,EncodePreset,Class,OrigRes,Name,FPS,BitDepth,CodedRes,QP,Bitrate(kbps)")
    csv.write("TestCfg,EncodeMethod,CodecName,EncodePreset,Class,Name,OrigRes,FPS,"\
              "BitDepth,CodedRes,QP,")
    csv.write("Bitrate(kbps)")
    for qty in QualityList:
        csv.write(',' + qty)
    csv.write(",EncT[s],DecT[s]")
    if UsePerfUtil:
        csv.write(",EncInstr,DecInstr,EncCycles,DecCycles")
    if EnableMD5:
        csv.write(",EncMD5,DecMD5")
    csv.write('\n')

    perframe_csv = open(perframe_csvfile, 'wt')

    perframe_csv.write("TestCfg,EncodeMethod,CodecName,EncodePreset,Class,Name,Res,FPS," \
                       "BitDepth,QP,POC,FrameType,Level,qindex,FrameSize")
    for qty in QualityList:
        if (qty != "Overall_PSNR" and qty != "Overall_APSNR" and not qty.startswith("APSNR")):
            perframe_csv.write(',' + qty)
    perframe_csv.write('\n')

    QPSet = QPs[test_cfg]
    for clip in clip_list:
        for qp in QPSet:
            bs, dec = GetBsReconFileName(EncodeMethod, CodecName, EncodePreset,
                                         test_cfg, clip, qp)
            if not os.path.exists(bs):
                print("%s is missing" % bs)
                missing.write("\n%s is missing" % bs)
                continue

            filesize = os.path.getsize(bs)
            bitrate = round((filesize * 8 * (clip.fps_num / clip.fps_denom)
                       / FrameNum[test_cfg]) / 1000.0, 6)

            quality, perframe_vmaf_log = GatherQualityMetrics(dec, Path_QualityLog)
            if not quality:
                print("%s is missing" % bs)
                missing.write("\n%s is missing" % bs)
                continue

            csv.write("%s,%s,%s,%s,%s,%s,%s,%.2f,%d,%s,%d,"
                      %(test_cfg,EncodeMethod,CodecName,EncodePreset,clip.file_class,
                        clip.file_name, str(clip.width)+'x'+str(clip.height),
                        clip.fps,clip.bit_depth,str(clip.width)+'x'+str(clip.height),qp))
            if (test_cfg == "STILL"):
                csv.write("%d"%filesize)
            else:
                csv.write("%f"%bitrate)

            for qty in quality:
                csv.write(",%f"%qty)
            
            if test_cfg == "RA" and EnableParallelGopEncoding:
                csv.write(",,,")
            elif UsePerfUtil:
                enc_time, dec_time, enc_instr, dec_instr, enc_cycles, dec_cycles = GatherInstrCycleInfo(bs, Path_TimingLog)
                csv.write(",%.2f,%.2f,%s,%s,%s,%s," % (enc_time,dec_time,enc_instr,dec_instr,enc_cycles,dec_cycles))
            elif EnableTimingInfo:
                enc_time, dec_time = GatherPerfInfo(bs, Path_TimingLog)
                csv.write(",%.2f,%.2f,"%(enc_time,dec_time))
            else:
                csv.write(",,,")
            if EnableMD5:
                enc_md5 = md5(bs)
                dec_md5 = md5(dec)
                csv.write("%s,%s"%(enc_md5, dec_md5))

            csv.write("\n")

            if (EncodeMethod == 'aom'):
                if test_cfg != "RA" or not EnableParallelGopEncoding:
                    enc_log = GetEncLogFile(bs, log_path)
                    GatherPerframeStat(test_cfg,EncodeMethod,CodecName,EncodePreset,clip,clip.file_name, clip.width,
                                        clip.height, qp,enc_log,perframe_csv, perframe_vmaf_log)
    csv.close()
    perframe_csv.close()
    Utils.Logger.info("finish export RD results to file.")
    return

def ParseArguments(raw_args):
    parser = argparse.ArgumentParser(prog='AV2CTCTestTest.py',
                                     usage='%(prog)s [options]',
                                     description='')
    parser.add_argument('-f', '--function', dest='Function', type=str,
                        required=True, metavar='',
                        choices=["clean", "encode", "summary", "concatenate", "decode"],
                        help="function to run: clean, encode, summary, concatenate, decode")
    parser.add_argument('-s', "--SaveMemory", dest='SaveMemory', type=bool,
                        default=True, metavar='',
                        help="save memory mode will delete most files in"
                             " intermediate steps and keeps only necessary "
                             "ones for RD calculation. It is false by default")
    parser.add_argument('-CmdOnly', "--LogCmdOnly", dest='LogCmdOnly', type=bool,
                        default=False, metavar='',
                        help="LogCmdOnly mode will only capture the command sequences"
                             "It is false by default")
    parser.add_argument('-l', "--LoggingLevel", dest='LogLevel', type=int,
                        default=3, choices=range(len(LogLevels)), metavar='',
                        help="logging level: 0:No Logging, 1: Critical, 2: Error,"
                             " 3: Warning, 4: Info, 5: Debug")
    parser.add_argument('-p', "--EncodePreset", dest='EncodePreset', type=str,
                        metavar='', help="EncodePreset: 0,1,2... for aom")
    parser.add_argument('-c', "--CodecName", dest='CodecName', type=str,
                        default='av2', choices=CodecNames, metavar='',
                        help="CodecName: av1, av2, hevc")
    parser.add_argument('-m', "--EncodeMethod", dest='EncodeMethod', type=str,
                        metavar='', help="EncodeMethod: aom, svt for av1, hm for hevc")
    if len(raw_args) == 1:
        parser.print_help()
        sys.exit(1)
    args = parser.parse_args(raw_args[1:])

    global Function, SaveMemory, LogLevel, EncodePreset, CodecName, EncodeMethod, LogCmdOnly
    Function = args.Function
    SaveMemory = args.SaveMemory
    LogLevel = args.LogLevel
    EncodePreset = args.EncodePreset
    CodecName = args.CodecName
    EncodeMethod = args.EncodeMethod
    LogCmdOnly = args.LogCmdOnly

######################################
# main
######################################
if __name__ == "__main__":
    #sys.argv = ["", "-f", "encode", "-c", "av2", "-m", "aom", "-p", "6", "--LogCmdOnly", "1"]
    #sys.argv = ["", "-f", "summary", "-c", "av2", "-m", "aom", "-p", "6"]
    #sys.argv = ["", "-f", "encode", "-c", "hevc", "-m", "hm", "-p", "0"] #, "--LogCmdOnly", "1"]
    #sys.argv = ["", "-f", "encode", "-c", "av1", "-m", "aom", "-p", "0"]  # , "--LogCmdOnly", "1"]
    ParseArguments(sys.argv)

    # preparation for executing functions
    setupWorkFolderStructure()
    if Function != 'clean':
        SetupLogging(LogLevel, LogCmdOnly, LoggerName, Path_CmdLog, Path_TestLog)

    # execute functions
    if Function == 'clean':
        CleanUp_workfolders()
    elif Function == 'encode':
        for test_cfg in TEST_CONFIGURATIONS:
            clip_list = CreateClipList(test_cfg)
            for clip in clip_list:
                if test_cfg == "RA" and EnableParallelGopEncoding:
                    Run_Parallel_Encode_Test(test_cfg, clip, CodecName, EncodeMethod, EncodePreset, LogCmdOnly)
                else:
                    Run_Encode_Test(test_cfg, clip, CodecName, EncodeMethod, EncodePreset, LogCmdOnly)
        if SaveMemory:
            Cleanfolder(Path_DecodedYuv)
    elif Function == 'concatenate':
        test_cfg = "RA"
        clip_list = CreateClipList(test_cfg)
        for clip in clip_list:
            if EnableParallelGopEncoding:
                Run_Concatenate_Test(test_cfg, clip, CodecName, EncodeMethod, EncodePreset, LogCmdOnly)
            else:
                Utils.Logger.error("concatenate function can only be used with Parallel Gop Encoding mode and RA configuration")
    elif Function == 'decode':
        test_cfg = "RA"
        clip_list = CreateClipList(test_cfg)
        for clip in clip_list:
            if EnableParallelGopEncoding:
                Run_Decode_Test(test_cfg, clip, CodecName, EncodeMethod, EncodePreset, LogCmdOnly)
            else :
                Utils.Logger.error("decode function can only be used with Parallel Gop Encoding mode and RA configuration")
    elif Function == 'summary':
        Missing = open("Missing.log", 'wt')
        for test_cfg in TEST_CONFIGURATIONS:
            clip_list = CreateClipList(test_cfg)
            GenerateSummaryRDDataFile(EncodeMethod, CodecName, EncodePreset,
                                      test_cfg, clip_list, Path_EncLog, Missing)
        Missing.close()
        Utils.Logger.info("RD data summary file generated")
    else:
        Utils.Logger.error("invalid parameter value of Function")
