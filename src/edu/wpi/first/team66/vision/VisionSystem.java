package edu.wpi.first.team66.vision;

import edu.wpi.first.team66.math.Math2;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;

public class VisionSystem {

    // expected color range, ~green
    public static final int HUE_LOW = 105;
    public static final int HUE_HIGH = 137;
    public static final int SAT_LOW = 230;
    public static final int SAT_HIGH = 255;
    public static final int VAL_LOW = 133;
    public static final int VAL_HIGH = 183;
    
    // ideal target aspect ratios
    private static final double VERTICAL_ASPECT_RATIO = 4.0 / 32.0;
    private static final double HORIZONTAL_ASPECT_RATIO = 23.5 / 4.0;
    
    // minimum area of particle to inspect
    private static final int AREA_MINIMUM = 150;
    
    // maximum number of particles to inspect
    private static final int MAX_PARTICLES = 8;

    // filter for minimum area of particle
    private final CriteriaCollection minimumAreaCriteria;
    
    // aspect ration matching criteria
    private static final double MIN_VASPECT_SCORE = 0.5;
    private static final double MIN_HASPECT_SCORE = 0.5;
    
    // target scoring criteria
    private static final double MIN_VOFFSET_SCORE = 0.2;
    private static final double MIN_HOFFSET_SCORE = 0.2;
    private static final double MIN_TOTAL_SCORE = 0.5;
    
    private final AxisCamera camera;
    
    public VisionSystem(AxisCamera camera) {
        this.camera = camera;
        
        minimumAreaCriteria = new CriteriaCollection();
        minimumAreaCriteria.addCriteria(
                NIVision.MeasurementType.IMAQ_MT_AREA,
                AREA_MINIMUM, 65535,
                false);
    }

    /**
     * Get an image from the camera and locate the best looking visual
     * target in view.
     * @return best target recognition, null if nothing meets criteria
     * @throws Exception 
     */
    public Target getTarget() throws Exception
    {
        ColorImage color = camera.getImage();
        
        BinaryImage threshold = color.thresholdHSV(
                HUE_LOW, HUE_HIGH,
                SAT_LOW, SAT_HIGH,
                VAL_LOW, VAL_HIGH);
        
        BinaryImage filtered =
                threshold.particleFilter(minimumAreaCriteria);
        
        ParticleAnalysisReport[] candidates =
                filtered.getOrderedParticleAnalysisReports(MAX_PARTICLES);
        
        int nHorizontal = 0;
        ParticleAnalysisReport[] horizontals =
                new ParticleAnalysisReport[MAX_PARTICLES];
        
        int nVertical = 0;
        ParticleAnalysisReport[] verticals =
                new ParticleAnalysisReport[MAX_PARTICLES];
        
        // bin horizontal and vertical particles
        for (int i = 0; i < candidates.length; i++)
        {
            ParticleAnalysisReport particle = candidates[i];
            
            if (isPossibleVertical(particle))
            {
                verticals[nVertical++] = particle;
            }
            else if (isPossibleHorizontal(particle))
            {
                horizontals[nHorizontal++] = particle;
            }
        }
        
        Target bestTarget = null;
        
        // try matching horizontals and verticals as a complete target
        // and find the best scoring pair
        for (int v = 0; v < nVertical; v++)
        {
            ParticleAnalysisReport vpart = verticals[v];
            
            for (int h = 0; h < nHorizontal; h++)
            {
                ParticleAnalysisReport hpart = horizontals[h];
                
                double verticalOffsetScore =
                        getVerticalOffsetScore(hpart, vpart);
                
                double leftOffsetScore =
                        getHorizontalOffsetScore(hpart, vpart, true);
                
                double rightOffsetScore =
                        getHorizontalOffsetScore(hpart, vpart, false);
                
                double horizontalOffsetScore =
                        Math.max(leftOffsetScore, rightOffsetScore);
                
                double totalScore =
                        verticalOffsetScore + horizontalOffsetScore;
                
                boolean hasMinimumScores =
                        verticalOffsetScore >= MIN_VOFFSET_SCORE &&
                        horizontalOffsetScore >= MIN_HOFFSET_SCORE &&
                        totalScore >= MIN_TOTAL_SCORE;
                
                if (null == bestTarget ||
                    (bestTarget.score < totalScore && hasMinimumScores))
                {
                    boolean isLeft = leftOffsetScore >= rightOffsetScore;
                    bestTarget = new Target(
                            vpart,
                            hpart,
                            totalScore,
                            isLeft);
                }
            }
        }
        
        return bestTarget;
    }
    
    /**
     * Given a particle, determine whether it meets criteria to be
     * considered a vertical target component
     * @param particle
     * @return 
     */
    private boolean isPossibleVertical(ParticleAnalysisReport particle)
    {
        double width = particle.boundingRectWidth;
        double height = particle.boundingRectHeight;
        double ratio = width / height;
        return Math2.hat(1, 1, 1, ratio / VERTICAL_ASPECT_RATIO) >=
                MIN_VASPECT_SCORE;
    }
    
    /**
     * Given a particle, determine whether it meets criteria to be
     * considered a horizontal target components
     * @param particle
     * @return 
     */
    private boolean isPossibleHorizontal(ParticleAnalysisReport particle)
    {
        double width = particle.boundingRectWidth;
        double height = particle.boundingRectHeight;
        double ratio = width / height;
        return Math2.hat(1, 1, 1, ratio / HORIZONTAL_ASPECT_RATIO) >=
                MIN_HASPECT_SCORE;
    }
    
    /**
     * Given a pair of horizontal and vertical particles, compute a
     * score [0, 1] corresponding to the likelihood of them composing
     * a single target, based on their relative vertical positions.
     * @param hpart horizontal particle
     * @param vpart vertical particle
     * @return 
     */
    private double getVerticalOffsetScore(
            ParticleAnalysisReport hpart,
            ParticleAnalysisReport vpart)
    {
        // hpart's vertical offset from center of vpart
        double vOffset = hpart.center_mass_y - vpart.center_mass_y;
        
        // vpart's height
        double vHeight = vpart.boundingRectHeight;
        
        if (vOffset <= 0)
        {
            // hpart not above vpart, invalid
            return 0.0;
        }
        else
        {
            // expected vOffset is a bit over vHeight / 2
            double expected = vHeight * 0.5 * 1.2;
            
            // (expected * 0.5, 0) to (expected, 1) to (expected * 1.5, 0)
            return Math2.hat(1, 0.5, 1, expected / vOffset);
        }
    }
    
    /**
     * Given a pair of horizontal and vertical particles, compute a
     * score [0, 1] corresponding to the likelihood of them composing
     * a single left or right target, based on their relative horizontal
     * positions.
     * @param hpart horizontal particle
     * @param vpart vertical particle
     * @param left true to compute likelihood of left target, false for right
     * @return 
     */
    private double getHorizontalOffsetScore(
            ParticleAnalysisReport hpart,
            ParticleAnalysisReport vpart,
            boolean left)
    {
        // get horizontal offset of hpart's center from vpart's center
        double hOffset = left ?
                vpart.center_mass_x - hpart.center_mass_x :
                hpart.center_mass_x - vpart.center_mass_x;
        
        // hPart's width
        double hWidth = hpart.boundingRectWidth;
        
        if (hOffset <= 0.0)
        {
            // hpart is in the opposite direction, invalid
            return 0.0;
        }
        else
        {
            // expected hOffset is a bit over hWidth / 2
            double expected = hWidth * 0.5 * 1.2;
            
            // (expected * 0.5, 0) to (expected, 1) to (expected * 1.5, 0)
            return Math2.hat(1, 0.5, 1, expected / hOffset);
        }
    }
    
    /**
     * Represents a detected target with both horizontal and vertical
     * components.
     */
    public static class Target
    {
        public final ParticleAnalysisReport vpart;
        
        public final ParticleAnalysisReport hpart;
        
        public final double score;
        
        public final boolean isLeft;
        
        public Target(
                ParticleAnalysisReport verticalParticle,
                ParticleAnalysisReport horizontalParticle,
                double score,
                boolean isLeft)
        {
            this.vpart = verticalParticle;
            this.hpart = horizontalParticle;
            this.score = score;
            this.isLeft = isLeft;
        }
    }
}
