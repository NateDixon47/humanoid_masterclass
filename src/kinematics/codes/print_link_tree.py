"""Print the child-sister link tree of the toy biped."""

from __future__ import annotations

from robot_model import setup_biped_robot


def main() -> None:
    links = setup_biped_robot()
    for link_id, link in enumerate(links):
        if link is None:
            continue
        print(
            f"{link_id:2d} {link.name:8s} "
            f"sister={link.sister:2d} child={link.child:2d} mother={link.mother:2d}"
        )


if __name__ == "__main__":
    main()

