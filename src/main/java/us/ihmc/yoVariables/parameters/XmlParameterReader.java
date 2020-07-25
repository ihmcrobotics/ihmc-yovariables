/*
 * Copyright 2017 Florida Institute for Human and Machine Cognition (IHMC)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package us.ihmc.yoVariables.parameters;

import java.io.IOException;
import java.io.InputStream;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;

import us.ihmc.yoVariables.parameters.xml.Parameter;
import us.ihmc.yoVariables.parameters.xml.Parameters;
import us.ihmc.yoVariables.parameters.xml.Registry;

/**
 * A parameter reader which manages parameter initialization using a XML file.
 */
public class XmlParameterReader extends AbstractParameterReader
{
   private static final String prefix = "[" + XmlParameterReader.class.getSimpleName() + "]:";

   private final boolean debug;

   private final Map<String, ParameterData> parameterValues = new HashMap<>();

   private final String rootNamespace;

   /**
    * Creates a parameter reader that will read the provided data streams.
    * <p>
    * If more than one data stream is passed to the reader multiple occurrences of the same parameter
    * will cause the parameter value to be overwritten with the new value.
    * </p>
    *
    * @param dataStreams the input streams from which the parameters' value can be read.
    * @throws IOException if something went wrong during parsing.
    */
   public XmlParameterReader(InputStream... dataStreams) throws IOException
   {
      this(false, null, dataStreams);
   }

   /**
    * Creates a parameter reader that will read the provided data streams.
    * <p>
    * If more than one data stream is passed to the reader multiple occurrences of the same parameter
    * will cause the parameter value to be overwritten with the new value.
    * </p>
    *
    * @param rootNamespace allows filtering the data in the data stream this is useful if the provided
    *                      data stream comes from a file that is used to load parameters in multiple
    *                      registries (e.g. controller and estimator).
    * @param dataStreams   the input streams from which the parameters' value can be read.
    * @throws IOException if something went wrong during parsing.
    */
   public XmlParameterReader(String rootNamespace, InputStream... dataStreams) throws IOException
   {
      this(false, rootNamespace, dataStreams);
   }

   /**
    * Creates a parameter reader that will read the provided data streams.
    * <p>
    * If more than one data stream is passed to the reader multiple occurrences of the same parameter
    * will cause the parameter value to be overwritten with the new value.
    * </p>
    *
    * @param debug       specifies whether to print additional information.
    * @param dataStreams the input streams from which the parameters' value can be read.
    * @throws IOException if something went wrong during parsing.
    */
   public XmlParameterReader(boolean debug, InputStream... dataStreams) throws IOException
   {
      this(debug, null, dataStreams);
   }

   /**
    * Creates a parameter reader that will read the provided data streams.
    * <p>
    * If more than one data stream is passed to the reader multiple occurrences of the same parameter
    * will cause the parameter value to be overwritten with the new value.
    * </p>
    *
    * @param debug         specifies whether to print additional information.
    * @param rootNamespace allows filtering the data in the data stream this is useful if the provided
    *                      data stream comes from a file that is used to load parameters in multiple
    *                      registries (e.g. controller and estimator).
    * @param dataStreams   the input streams from which the parameters' value can be read.
    * @throws IOException if something went wrong during parsing.
    */
   public XmlParameterReader(boolean debug, String rootNamespace, InputStream... dataStreams) throws IOException
   {
      this.debug = debug;
      this.rootNamespace = rootNamespace;

      for (InputStream dataStream : dataStreams)
      {
         readStream(dataStream, false);
      }
   }

   /**
    * Will overwrite parameters in the parameter reader with parameters specified in the provided
    * streams. This method will throw a {@link RuntimeException} if any parameter that needs to be
    * overwritten does not exist.
    *
    * @param overwriteParameters the input streams from which the parameters' value to override can be
    *                            read.
    * @throws IOException if something went wrong during parsing.
    */
   public void overwrite(InputStream... overwriteParameters) throws IOException
   {
      for (InputStream dataStream : overwriteParameters)
      {
         readStream(dataStream, true);
      }
   }

   private void readStream(InputStream data, boolean forceOverwrite) throws IOException
   {
      try
      {
         JAXBContext jaxbContext = JAXBContext.newInstance(Parameters.class);
         Unmarshaller jaxbUnmarshaller = jaxbContext.createUnmarshaller();

         Parameters parameterRoot = (Parameters) jaxbUnmarshaller.unmarshal(data);
         if (parameterRoot.getRegistries() != null)
         {
            for (Registry registry : parameterRoot.getRegistries())
            {
               if (rootNamespace == null || registry.getName().equals(rootNamespace))
               {
                  addRegistry(registry.getName(), registry, forceOverwrite);
               }
            }
         }
      }
      catch (JAXBException e)
      {
         throw new IOException(e);
      }
   }

   private void addRegistry(String path, Registry registry, boolean forceOverwrite)
   {
      if (registry.getParameters() != null)
      {
         for (Parameter param : registry.getParameters())
         {
            String name = path + "." + param.getName();
            ParameterData data = new ParameterData(param.getValue(), param.getMin(), param.getMax());
            if (parameterValues.put(name, data) != null)
            {
               if (debug)
               {
                  System.out.println(prefix + " overwriting " + param.getName());
               }
            }
            else if (forceOverwrite)
            {
               throw new RuntimeException(prefix + " trying to overwrite parameter " + param.getName() + " but it does not exist.");
            }
         }
      }

      if (registry.getRegistries() != null)
      {
         for (Registry child : registry.getRegistries())
         {
            String childPath = path + "." + child.getName();
            addRegistry(childPath, child, forceOverwrite);
         }
      }
   }

   @Override
   protected Map<String, ParameterData> getValues()
   {
      return Collections.unmodifiableMap(parameterValues);
   }
}
