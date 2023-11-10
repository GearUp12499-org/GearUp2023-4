package org.firstinspires.ftc.teamcode.utilities;

import android.util.Pair;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

public class CollectionUtils {
    public static <T> List<Pair<T, T>> pairs(List<T> source) {
        if (source.size() < 2) {
            return new ArrayList<>();
        }
        HashSet<Pair<T, T>> used = new HashSet<>();
        List<Pair<T, T>> result = new ArrayList<>();
        for (T pairItemOne : source) {
            for (T pairItemTwo : source) {
                // Intentional exact reference comparison
                if (pairItemOne == pairItemTwo) {
                    continue;
                }
                // don't use reversed pairs
                if (used.contains(new Pair<>(pairItemTwo, pairItemOne))) {
                    continue;
                }
                Pair<T, T> the = new Pair<>(pairItemOne, pairItemTwo);
                used.add(the);
                result.add(the);
            }
        }
        return result;
    }
}
