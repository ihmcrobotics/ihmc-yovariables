package us.ihmc.yoVariables.dataBuffer;

import java.util.ArrayList;

/**
 * @author jcarff
 */
public class KeyPoints
{
	ArrayList<Integer> keyPoints = new ArrayList<Integer>();
	boolean toggleKeyPoints = false;
	boolean DEBUG = false;

	public boolean setKeyPoint(int time)
	{
		for (int i = 0; i < keyPoints.size(); i++)
		{
			if (keyPoints.get(i) == time)
			{
				removeKeyPoint(time);

				return false;
			}

			if (keyPoints.get(i) > time)
			{
				keyPoints.add(i, time);

				return true;
			}
		}

		keyPoints.add(time);

		return true;
	}

	public void removeKeyPoint(int time)
	{
		for (int i = 0; i < keyPoints.size(); i++)
		{
			if (keyPoints.get(i) == time)
			{
				keyPoints.remove(i);

				break;
			}
		}
	}

	public int getNextTime(int time)
	{
		for (int i = 0; i < keyPoints.size(); i++)
		{
			if (keyPoints.get(i) > time)
			{
				return keyPoints.get(i);
			}
		}

		if (keyPoints.size() > 0)
		{
			return keyPoints.get(0);
		}

		return time;
	}

	public int getPreviousTime(int time)
	{
		for (int i = keyPoints.size() - 1; i >= 0; i--)
		{
			if(DEBUG)
				System.out.println(keyPoints.get(i));
			if (keyPoints.get(i) < time)
			{
				return keyPoints.get(i);
			}
		}

		if (keyPoints.size() > 0)
		{
			return keyPoints.get(keyPoints.size() - 1);
		}

		return time;
	}

	public void trim(int inPoint, int outPoint)
	{
		for (int i = 0; i < keyPoints.size(); i++)
		{
			if(DEBUG)
			 	System.out.println(keyPoints.get(i));
			if (inPoint < outPoint)
			{
				if ((keyPoints.get(i) < inPoint) || (keyPoints.get(i) > outPoint))
				{
					keyPoints.remove(i);
					i--;
				}
			}
			else
			{
				if ((keyPoints.get(i) < inPoint) && (keyPoints.get(i) > outPoint))
				{
					keyPoints.remove(i);
					i--;
				}
			}
		}
	}

	public void setUseKeyPoints(boolean use)
	{
		toggleKeyPoints = use;
	}

	public boolean useKeyPoints()
	{
		return toggleKeyPoints;
	}

	public ArrayList<Integer> getPoints()
	{
		return keyPoints;
	}
}