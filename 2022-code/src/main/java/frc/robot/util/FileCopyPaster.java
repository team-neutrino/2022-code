package frc.robot.util;

import java.io.File;  
import java.io.IOException; 
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;

public class FileCopyPaster {

  private String m_src; 
  private String m_dst;
  private File m_srcFile;  
  private File m_dstFile;                     
  private List<File> m_srcFileArray;

  private String m_username;
  private boolean m_isPlugin;

  public FileCopyPaster(String src, String username, boolean isPlugin) {
    m_src = src;                           
    m_dst = URLCreator(username, isPlugin);
    m_srcFile = new File(m_src);                           
    m_dstFile = new File(m_dst);
    m_srcFileArray = new ArrayList<File>(); 

    m_username = username;      
    m_isPlugin = isPlugin;                    

    String[] srcNames = m_srcFile.list();
    for (String srcName : srcNames) {
      m_srcFileArray.add(new File(srcName));
    }
    
    copyPaste();
  }

  public static String URLCreator(String username, boolean isPlugin) {
    String baseURL = "C:/Users/" + username + "/Shuffleboard";
    return (isPlugin) ? (baseURL + "/plugins/") : 
                        (baseURL + "/themes/");
  }

  public void copyPaste() {
    try {
      for (File file : m_srcFileArray) {
        String filename = file.getName(); 
        String fullpath = URLCreator(m_username, m_isPlugin) + filename.substring(0, filename.length()-4);
        (new File(fullpath)).mkdir();
        Files.copy(
          (new File(m_src + "/" + file.getName())).toPath(), 
          (new File(fullpath + "/" + file.getName())).toPath(), 
          StandardCopyOption.REPLACE_EXISTING);
      }
    } catch(IOException e) {
      System.out.println("error");
      e.printStackTrace();
    }

  }

  public static void main(String[] args) {
    // use this for testing, keep
    // new FileCopyPaster("src/main/java/frc/robot/util/themes/", "neutrino", false);
  }
}