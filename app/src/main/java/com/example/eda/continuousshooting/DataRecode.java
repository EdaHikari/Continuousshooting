package com.example.eda.continuousshooting;

import android.content.Context;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.io.File;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

public class DataRecode {
    File mFile;

    DataRecode(Context context, String filename){
        mFile = new File(context.getExternalFilesDir(null), filename+".xml");
    }
    public Mat loadData(String dataName){
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        try {
            //ファイルの読み込み
            DocumentBuilder builder = factory.newDocumentBuilder();
            Document doc = builder.parse(mFile);
            Element root = doc.getDocumentElement();
            NodeList rootNodeList = root.getChildNodes();
            Element info = null;
            for (int i = 0; i < rootNodeList.getLength(); i++) {
                if (rootNodeList.item(i).getNodeType() == Node.ELEMENT_NODE) {
                    if(rootNodeList.item(i).getNodeName().equals(dataName)) {
                        info = (Element)rootNodeList.item(i);
                    }
                }
            }if (info == null) return new MatOfPoint2f();

            String type = info.getAttribute("type");
            int cols = Integer.valueOf(info.getAttribute("cols"));
            int rows = Integer.valueOf(info.getAttribute("rows"));
            int channels = Integer.valueOf(info.getAttribute("channels"));

            String Datastringlong = info.getAttribute("matData");
            Datastringlong = Datastringlong.replaceAll("\\[", "").replaceAll("\\]", "").trim();
            String[] Datastringshort = Datastringlong.split(";");

            Mat mat = new Mat(rows, cols, CvType.CV_32FC(channels));
            for (int row = 0; row < rows; row++) {
                String[] matDataSplitted = Datastringshort[row].split(",");

                for (int col = 0; col < cols; col++) {
                    float[] matData = new float[channels];
                    for (int channel = 0; channel < channels; channel++) {
                        matData[channel] = Float.valueOf(matDataSplitted[col * channels + channel].trim());
                    }
                    mat.put(row, col, matData);
                }
            }
            return mat;
        } catch (Exception e) {
            e.printStackTrace();
        }
        return new MatOfPoint2f();
    }

    //保存するデータがmatだった場合
    public void writeData(String dataName,Mat mat){
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        try {
            DocumentBuilder builder = factory.newDocumentBuilder();
            Document doc = builder.newDocument();
            Node root = doc.createElement("DataRoot");
            doc.appendChild(root);

            //データの形式
            Element info = doc.createElement(dataName);
            info.setAttribute("type","Mat");
            info.setAttribute("rows",Integer.toString(mat.rows()));
            info.setAttribute("cols",Integer.toString(mat.cols()));
            info.setAttribute("channels",Integer.toString(mat.channels()));
            info.setAttribute("matData",mat.dump());
            root.appendChild(info);

            //ファイルの形式
            TransformerFactory tfFactory = TransformerFactory.newInstance();
            Transformer tf = tfFactory.newTransformer();

            tf.setOutputProperty("indent", "yes");
            tf.setOutputProperty("encoding", "UTF-8");

            tf.transform(new DOMSource(doc), new StreamResult(mFile));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void appendData(String dataName,Mat mat){
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        try {
            DocumentBuilder builder = factory.newDocumentBuilder();
            Document doc = builder.parse(mFile);
            Node root = doc.getDocumentElement();

            //データの形式
            Element info = doc.createElement(dataName);
            info.setAttribute("type","Mat");
            info.setAttribute("rows",Integer.toString(mat.rows()));
            info.setAttribute("cols",Integer.toString(mat.cols()));
            info.setAttribute("channels",Integer.toString(mat.channels()));
            info.setAttribute("matData",mat.dump());
            root.appendChild(info);

            //ファイルの形式
            TransformerFactory tfFactory = TransformerFactory.newInstance();
            Transformer tf = tfFactory.newTransformer();

            tf.setOutputProperty("indent", "yes");
            tf.setOutputProperty("encoding", "UTF-8");

            tf.transform(new DOMSource(doc), new StreamResult(mFile));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
