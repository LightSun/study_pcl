cd /d %~dp0
SET RES_DIR=E:\study\github\mine\study_pcl\pcl1\res
pcl_viewer_release.exe -multiview 1 %RES_DIR%/table_scene_lms400.pcd %RES_DIR%/table_scene_lms400_downsampled.pcd
@PAUSE
