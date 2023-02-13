use num_traits::identities::Zero;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::fmt::Debug;
use std::hash::Hash;

pub fn astar<N, C, FN, IN, FH, FS>(
  start: &N,
  mut successors: FN,
  mut heuristic: FH,
  mut success: FS,
) -> Option<(Vec<N>, C)>
where
  N: Eq + Hash + Clone + Ord,
  C: Zero + Ord + Copy,
  FN: FnMut(&N) -> IN,
  IN: IntoIterator<Item = (N, C)>,
  FH: FnMut(&N) -> C,
  FS: FnMut(&N) -> bool,
{
  let mut open_set = BinaryHeap::new();
  let mut came_from = HashMap::new();
  let mut g_score = HashMap::new();
  let mut f_score = HashMap::new();
  let mut visited = HashSet::new();
  let mut count = 0;

  g_score.insert(start.clone(), C::zero());
  f_score.insert(start.clone(), heuristic(start));

  open_set.push((Reverse(f_score[start]), count, start.clone()));

  while let Some((_, _, current)) = open_set.pop() {
    if success(&current) {
      let path = reconstruct_path::<N, C>(&came_from, current.clone());
      return Some((path, g_score[&current]));
    }

    visited.insert(current.clone());

    for (neighbor, cost) in successors(&current) {
      let tentative_g_score = *g_score.get(&current).unwrap() + cost;

      if visited.contains(&neighbor)
        && tentative_g_score >= *g_score.get(&neighbor).unwrap_or(&C::zero())
      {
        continue;
      }

      if !visited.contains(&neighbor)
        || tentative_g_score < *g_score.get(&neighbor).unwrap_or(&C::zero())
      {
        came_from.insert(neighbor.clone(), current.clone());
        g_score.insert(neighbor.clone(), tentative_g_score);
        f_score.insert(neighbor.clone(), tentative_g_score + heuristic(&neighbor));

        open_set.push((Reverse(f_score[&neighbor]), count, neighbor.clone()));
        count += 1;
      }
    }
  }

  None
}

fn reconstruct_path<N, C>(came_from: &HashMap<N, N>, mut current: N) -> Vec<N>
where
  N: Eq + Hash + Clone,
{
  let mut path = vec![current.clone()];

  while let Some(prev) = came_from.get(&current) {
    path.push(prev.clone());
    current = prev.clone();
  }

  path.reverse();

  path
}
