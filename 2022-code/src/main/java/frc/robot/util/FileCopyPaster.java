package frc.robot.util;

import java.io.File;  
import java.io.IOException; 
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;

public class FileCopyPaster {

  private String m_src; // rmbr CALL THESE FOLDERS!!
  private String m_dst;
  private File m_srcFile;
  private File m_dstFile;
  private List<File> m_srcFileArray;

  public FileCopyPaster(String src, String dst) {
    m_src = src;
    m_dst = dst;
    m_srcFile = new File(m_src);
    m_dstFile = new File(m_dst);
    m_srcFileArray = new ArrayList<File>();

    String[] srcNames = m_srcFile.list();
    for (String srcName : srcNames) {
      m_srcFileArray.add(new File(srcName));
      System.out.println(srcName);
    }
    
    copyPaste();
  }

  public void copyPaste() {
    
    try {
      for (File file : m_srcFileArray) {
        Files.copy(
          (new File(m_src + file.getName())).toPath(), 
          (new File(m_dst + file.getName())).toPath(), 
          StandardCopyOption.REPLACE_EXISTING);
      }
    } catch(IOException e) {
      System.out.println("error");
      e.getStackTrace();
    }

  }

    /*
    Path dir;
    List<Path> files;
    try (DirectoryStream<Path> stream = Files.newDirectoryStream(dir)) {
      for (Path file: stream) {
        files.add(file);
      }
    }
    for(Path file : files) {
        Files.copy(file, new File(m_destination + file.getName(-1)).toPath(),
        StandardCopyOption.REPLACE_EXISTING);
    }*/
  
  /*
    try {
      File myObj = new File(m_dir + m_file);
      if (myObj.createNewFile()) {
        System.out.println("File created at " + myObj.getName());
      } else {
        System.out.println("File already exists");
      }
    } catch (IOException e) {
      System.out.println("error, not important");
      e.printStackTrace();
    }
    */


  public static void main(String[] args) {
    //String testDir = "src/main/java/frc/robot/util/";
    FileCopyPaster fileCopyPaster = new FileCopyPaster("src/main/java/frc/robot/util/srcFolder/", "src/main/java/frc/robot/util/themes/");
    new FileCopyPaster("src/main/java/frc/robot/util/srcFolder/", "C:/Users/david/Shuffleboard/themes/");
  }
}