package org.firstinspires.ftc.pedroext.fusion;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * The set of AprilTag IDs that are valid for field localization this season — a
 * <b>config input</b>, because some tags must NOT be used to localize.
 *
 * <p>For the 2025–26 DECODE season this means the goal-region localization tags
 * only; the <b>Obelisk</b> tags (IDs 21, 22, 23) encode the randomized motif and
 * are NOT localization tags, so they are excluded by {@link #decodeDefault()}.
 * Confirm the exact valid IDs and their field poses against the current Game
 * Manual before relying on this on hardware.
 */
public final class TagMap {

    private final Set<Integer> localizationTagIds;

    public TagMap(Set<Integer> localizationTagIds) {
        this.localizationTagIds = Collections.unmodifiableSet(new HashSet<>(localizationTagIds));
    }

    /**
     * DECODE (2025–26) default: the goal localization tags (20 and 24), explicitly
     * excluding the Obelisk motif tags 21–23. ADJUST to your field/season.
     */
    public static TagMap decodeDefault() {
        Set<Integer> ids = new HashSet<>();
        ids.add(20);
        ids.add(24);
        return new TagMap(ids);
    }

    public boolean isLocalizationTag(int tagId) {
        return localizationTagIds.contains(tagId);
    }

    public Set<Integer> ids() {
        return localizationTagIds;
    }
}
