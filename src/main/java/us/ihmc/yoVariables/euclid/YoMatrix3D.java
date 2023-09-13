package us.ihmc.yoVariables.euclid;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoMatrix3D implements Matrix3DBasics, Settable<Matrix3D>
{
   private final String namePrefix;
   private final String nameSuffix;
   private final YoDouble m00, m01, m02, m10, m11, m12, m20, m21, m22;

   public YoMatrix3D(String namePrefix, YoRegistry registry)
   {
      this(namePrefix, "", registry);
   }

   public YoMatrix3D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      m00 = new YoDouble(namePrefix + "M00" + nameSuffix, registry);
      m01 = new YoDouble(namePrefix + "M01" + nameSuffix, registry);
      m02 = new YoDouble(namePrefix + "M02" + nameSuffix, registry);
      m10 = new YoDouble(namePrefix + "M10" + nameSuffix, registry);
      m11 = new YoDouble(namePrefix + "M11" + nameSuffix, registry);
      m12 = new YoDouble(namePrefix + "M12" + nameSuffix, registry);
      m20 = new YoDouble(namePrefix + "M20" + nameSuffix, registry);
      m21 = new YoDouble(namePrefix + "M21" + nameSuffix, registry);
      m22 = new YoDouble(namePrefix + "M22" + nameSuffix, registry);
   }

   @Override
   public void set(Matrix3D other)
   {
      this.m00.set(other.getM00());
      this.m01.set(other.getM01());
      this.m02.set(other.getM02());
      this.m10.set(other.getM10());
      this.m11.set(other.getM11());
      this.m12.set(other.getM12());
      this.m20.set(other.getM20());
      this.m21.set(other.getM21());
      this.m22.set(other.getM22());
   }

   @Override
   public void setM00(double m00)
   {
      this.m00.set(m00);

   }

   @Override
   public void setM01(double m01)
   {
      this.m01.set(m01);
   }

   @Override
   public void setM02(double m02)
   {
      this.m02.set(m02);
   }

   @Override
   public void setM10(double m10)
   {
      this.m10.set(m10);
   }

   @Override
   public void setM11(double m11)
   {
      this.m11.set(m11);
   }

   @Override
   public void setM12(double m12)
   {
      this.m12.set(m12);
   }

   @Override
   public void setM20(double m20)
   {
      this.m20.set(m20);
   }

   @Override
   public void setM21(double m21)
   {
      this.m21.set(m21);
   }

   @Override
   public void setM22(double m22)
   {
      this.m22.set(m22);
   }

   @Override
   public double getM00()
   {
      return m00.getValue();
   }

   @Override
   public double getM01()
   {
      return m01.getValue();
   }

   @Override
   public double getM02()
   {
      return m02.getValue();
   }

   @Override
   public double getM10()
   {
      return m10.getValue();
   }

   @Override
   public double getM11()
   {
      return m11.getValue();
   }

   @Override
   public double getM12()
   {
      return m12.getValue();
   }

   @Override
   public double getM20()
   {
      return m20.getValue();
   }

   @Override
   public double getM21()
   {
      return m21.getValue();
   }

   @Override
   public double getM22()
   {
      return m22.getValue();
   }

   public String getNamePrefix()
   {
      return namePrefix;
   }

   public String getNameSuffix()
   {
      return nameSuffix;
   }

   public void attachVariableChangedListener(YoVariableChangedListener variableChangedListener)
   {
      m00.addListener(variableChangedListener);
      m01.addListener(variableChangedListener);
      m02.addListener(variableChangedListener);
      m10.addListener(variableChangedListener);
      m11.addListener(variableChangedListener);
      m12.addListener(variableChangedListener);
      m20.addListener(variableChangedListener);
      m21.addListener(variableChangedListener);
      m21.addListener(variableChangedListener);
   }
}
